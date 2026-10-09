#!/usr/bin/env python3
"""Incremental, collision-aware 3D voxel frontier exploration for one UAV."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import time
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField, Range
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from teleop import Teleop
from voxel_occupancy import FREE, OCCUPIED, UNKNOWN, ProbabilisticVoxelMap, is_frontier, ray_voxels

Voxel = tuple[int, int, int]
NEIGHBORS_6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
NEIGHBORS_26 = tuple(
    (x, y, z)
    for x in (-1, 0, 1)
    for y in (-1, 0, 1)
    for z in (-1, 0, 1)
    if (x, y, z) != (0, 0, 0)
)


def yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def rotation_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.asarray((
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    ), dtype=np.float64)


class VoxelExplorer(Teleop):
    def __init__(self, args) -> None:
        super().__init__("/drone_1", "hospital_voxel_explorer")
        self.args = args
        self.occupancy = ProbabilisticVoxelMap()
        # Keep this alias for compact serialization and confidence display.
        self.scores = self.occupancy.evidence
        self.changed_voxels: set[Voxel] = set()
        self.map_revision = 0
        self.cached_revision = -1
        self.cached_sets: tuple[set[Voxel], set[Voxel], set[Voxel]] | None = None
        self.cloud: PointCloud2 | None = None
        self.odom: Odometry | None = None
        self.odom_received_at = 0.0
        self.planning_frame = "icp_odom"
        self.down = math.inf
        self.up = math.inf
        self.last_cloud_stamp = None
        self.last_integrate = 0.0
        self.last_map_growth_at = time.monotonic()
        self.last_visualize = 0.0
        self.target: Voxel | None = None
        self.target_best_distance = math.inf
        self.target_progress_at = time.monotonic()
        self.path: list[Voxel] = []
        self.path_index = 0
        self.excluded: list[Voxel] = []
        self.breadcrumbs: list[Voxel] = []
        self.tf = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.create_subscription(PointCloud2, "/slam/lidar/points", self._cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/slam/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(Range, "/range/down", lambda m:setattr(self, "down", m.range), 10)
        self.create_subscription(Range, "/range/up", lambda m:setattr(self, "up", m.range), 10)
        self.path_pub = self.create_publisher(RosPath, "/exploration/planned_path_3d", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/exploration/voxel_markers", 1)
        self.voxel_snapshot_pub = self.create_publisher(
            PointCloud2, "/exploration/occupied_voxels", 1
        )
        self.last_frontier_voxels = 0
        self.last_frontier_clusters = 0
        self.last_valid_frontier_clusters = 0
        # A sparse, standard ROS representation for a future map-fusion node.
        # Each point carries absolute per-robot log-odds and stable state.
        # A fusion node can track each robot's latest value and combine them in
        # a shared frame without retransmitting a dense map.
        self.delta_pub = self.create_publisher(
            PointCloud2, "/drone_1/exploration/voxel_delta", QoSProfile(depth=10)
        )
        metadata_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.metadata_pub = self.create_publisher(
            String, "/drone_1/exploration/voxel_metadata", metadata_qos
        )
        self.publish_metadata()

    def _cloud(self, message): self.cloud = message
    def _odom(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()
        if message.header.frame_id:
            self.planning_frame = message.header.frame_id

    def voxel(self, point) -> Voxel:
        r = self.args.voxel_size
        return tuple(int(math.floor(float(value)/r)) for value in point)

    def centre(self, voxel: Voxel) -> np.ndarray:
        return (np.asarray(voxel, dtype=np.float64)+0.5)*self.args.voxel_size

    def within_bounds(self, voxel: Voxel) -> bool:
        x,y,z = self.centre(voxel)
        return (
            self.args.min_x <= x <= self.args.max_x
            and self.args.min_y <= y <= self.args.max_y
            and self.args.min_z <= z <= self.args.max_z
        )

    def body_pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Continuous SLAM-odometry pose used for planning and control.

        RTAB-Map's map->odom correction can jump after loop closure. Using that
        corrected pose directly as controller feedback caused the apparent
        multi-metre jumps seen in the first run. The odometry frame is locally
        continuous; TF still relates it to ``map`` for RViz and map fusion.
        """
        if self.odom is None or time.monotonic()-self.odom_received_at > 1.0:
            return None
        p = self.odom.pose.pose.position
        return np.asarray((p.x, p.y, p.z), dtype=np.float64), rotation_matrix(self.odom.pose.pose.orientation)

    def position(self) -> np.ndarray | None:
        pose = self.body_pose()
        return None if pose is None else pose[0]

    def simulation_stamp(self):
        """Use the SLAM clock domain for every RViz-visible message."""
        if self.odom is not None:
            return self.odom.header.stamp
        return self.get_clock().now().to_msg()

    def integrate_cloud(self) -> bool:
        now = time.monotonic()
        previous_known_count = len(self.occupancy.states)
        if self.cloud is None or now-self.last_integrate < self.args.map_period:
            return False
        pose = self.body_pose()
        if pose is None:
            return False
        message = self.cloud
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.last_cloud_stamp or message.point_step < 12:
            return False
        self.last_cloud_stamp, self.last_integrate = stamp, now
        count = min(message.width*message.height, len(message.data)//message.point_step)
        # Copy out of the ROS message before another callback can release or
        # replace its backing storage while this relatively long update runs.
        points = np.ndarray(
            (count, 3), dtype=np.float32, buffer=message.data,
            strides=(message.point_step, 4),
        ).copy()
        points = points[np.isfinite(points).all(axis=1)]
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges >= 0.50) & (ranges <= self.args.sensor_range)]
        if not len(points):
            return False
        # One representative return per endpoint voxel suppresses oversampling
        # and bounds ray-tracing work independently of LiDAR channel count.
        sensor_voxels = np.floor(points/self.args.voxel_size).astype(np.int32)
        _, unique = np.unique(sensor_voxels, axis=0, return_index=True)
        points = points[np.sort(unique)]
        if len(points) > self.args.max_rays:
            points = points[np.linspace(0, len(points)-1, self.args.max_rays, dtype=np.int32)]
        body_origin, matrix = pose
        # Static body->LiDAR transform from the simulation sensor rig.
        origin = body_origin + matrix @ np.asarray((0.0, 0.0, 0.09), dtype=np.float64)
        endpoints = points.astype(np.float64) @ matrix.T + origin
        hit_voxels: set[Voxel] = set()
        miss_voxels: set[Voxel] = set()
        for endpoint in endpoints:
            delta = endpoint-origin
            distance = float(np.linalg.norm(delta))
            if distance < 0.5:
                continue
            hit = self.voxel(endpoint)
            if self.within_bounds(hit):
                hit_voxels.add(hit)
            # Exact grid traversal excludes the endpoint, so free-space
            # evidence cannot erase the surface return from the same scan.
            for cell in ray_voxels(tuple(origin), tuple(endpoint), self.args.voxel_size):
                if self.within_bounds(cell):
                    miss_voxels.add(cell)
        miss_voxels.difference_update(hit_voxels)
        # Guarantee a connected free seed around the vehicle despite sparse
        # beam geometry. This volume has already been physically occupied.
        start = self.voxel(origin)
        forced_free = set()
        for dx,dy,dz in NEIGHBORS_26+((0,0,0),):
            cell=(start[0]+dx,start[1]+dy,start[2]+dz)
            if self.within_bounds(cell):
                forced_free.add(cell)
        self.changed_voxels.update(self.occupancy.update(miss_voxels, hit_voxels, forced_free))
        self.publish_delta()
        if self.occupancy.last_state_changes:
            self.map_revision += 1
        if len(self.occupancy.states) > previous_known_count:
            self.last_map_growth_at = now
        return True

    def publish_metadata(self) -> None:
        message = String()
        message.data = json.dumps({
            "schema": "sparse_quantized_log_odds_v2",
            "robot_id": "drone_1",
            "frame": self.planning_frame,
            "resolution_m": self.args.voxel_size,
            "score_range": [self.occupancy.config.minimum, self.occupancy.config.maximum],
            "occupied_enter_exit": [self.occupancy.config.occupied_enter, self.occupancy.config.occupied_exit],
            "free_enter_exit": [self.occupancy.config.free_enter, self.occupancy.config.free_exit],
            "bounds_m": {"min":[self.args.min_x,self.args.min_y,self.args.min_z],"max":[self.args.max_x,self.args.max_y,self.args.max_z]},
            "delta_encoding": "PointCloud2 xyz=float32 centre, intensity=float32 quantized_log_odds, state=float32 {-1,0,1}; per-robot values are absolute",
        }, separators=(",", ":"))
        self.metadata_pub.publish(message)

    def publish_delta(self) -> None:
        if not self.changed_voxels:
            return
        cells = list(self.changed_voxels)
        data = np.empty((len(cells), 5), dtype=np.float32)
        data[:, :3] = (np.asarray(cells, dtype=np.float32)+0.5)*self.args.voxel_size
        data[:, 3] = np.asarray([self.scores[cell] for cell in cells], dtype=np.float32)
        data[:, 4] = np.asarray([self.occupancy.state(cell) for cell in cells], dtype=np.float32)
        message = PointCloud2()
        message.header.frame_id = self.planning_frame
        message.header.stamp = self.simulation_stamp()
        message.height = 1
        message.width = len(cells)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="state", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 20
        message.row_step = message.point_step*message.width
        message.is_dense = True
        message.data = data.tobytes()
        self.delta_pub.publish(message)
        self.changed_voxels.clear()

    def map_sets(self) -> tuple[set[Voxel], set[Voxel], set[Voxel]]:
        if self.cached_revision == self.map_revision and self.cached_sets is not None:
            return self.cached_sets
        free={cell for cell,state in self.occupancy.states.items() if state == FREE}
        occupied={cell for cell,state in self.occupancy.states.items() if state == OCCUPIED}
        inflated=set(occupied)
        hr=math.ceil(self.args.horizontal_radius/self.args.voxel_size)
        vr=math.ceil(self.args.vertical_radius/self.args.voxel_size)
        for x,y,z in occupied:
            for dx in range(-hr,hr+1):
                for dy in range(-hr,hr+1):
                    for dz in range(-vr,vr+1):
                        if (dx/hr)**2+(dy/hr)**2+(dz/max(1,vr))**2 <= 1.0:
                            inflated.add((x+dx,y+dy,z+dz))
        self.cached_sets = (free, occupied, free-inflated)
        self.cached_revision = self.map_revision
        return self.cached_sets

    def reachable(self, safe: set[Voxel], start: Voxel) -> set[Voxel]:
        if start not in safe:
            candidates=sorted(safe,key=lambda c:sum((c[i]-start[i])**2 for i in range(3)))[:1]
            if not candidates:
                return set()
            start=candidates[0]
        found={start}; queue=deque([start])
        while queue:
            current=queue.popleft()
            for d in NEIGHBORS_6:
                nxt=tuple(current[i]+d[i] for i in range(3))
                if nxt in safe and nxt not in found:
                    found.add(nxt); queue.append(nxt)
        return found

    def astar(self, safe: set[Voxel], start: Voxel, goal: Voxel) -> list[Voxel]:
        if start not in safe:
            nearest=sorted(safe,key=lambda c:sum((c[i]-start[i])**2 for i in range(3)))[:1]
            if not nearest: return []
            start=nearest[0]
        queue=[(0.0,start)]; came={}; cost={start:0.0}; visited=set()
        while queue and len(visited)<self.args.max_astar_voxels:
            _,current=heapq.heappop(queue)
            if current in visited: continue
            visited.add(current)
            if current==goal:
                path=[current]
                while current in came: current=came[current]; path.append(current)
                return path[::-1]
            for d in NEIGHBORS_26:
                nxt=tuple(current[i]+d[i] for i in range(3))
                if nxt not in safe: continue
                # Disallow diagonal corner cutting through an occupied axis cell.
                if sum(value!=0 for value in d)>1 and any(
                    tuple(current[i]+(d[i] if i==axis else 0) for i in range(3)) not in safe
                    for axis in range(3) if d[axis]
                ): continue
                edge=math.sqrt(d[0]*d[0]+d[1]*d[1]+1.4*d[2]*d[2])
                new=cost[current]+edge
                if new<cost.get(nxt,math.inf):
                    cost[nxt]=new; came[nxt]=current
                    heuristic=math.sqrt(sum((goal[i]-nxt[i])**2 for i in range(3)))
                    heapq.heappush(queue,(new+heuristic,nxt))
        return []

    def choose_frontier(self) -> bool:
        position=self.position()
        if position is None: return False
        free,occupied,safe=self.map_sets(); start=self.voxel(position)
        connected=self.reachable(safe,start)
        if not connected:
            print("3D planner has no clearance-valid free component",flush=True); return False
        frontiers=[
            cell for cell in connected
            if is_frontier(
                cell, self.occupancy.states, self.within_bounds, NEIGHBORS_6,
                self.args.frontier_unknown_neighbors, 2,
            )
        ]
        # Multi-resolution clustering: one strongest representative per 0.9 m cube.
        clusters: dict[Voxel,list[Voxel]]=defaultdict(list)
        scale=max(2,round(0.9/self.args.voxel_size))
        for cell in frontiers: clusters[tuple(value//scale for value in cell)].append(cell)
        self.last_frontier_voxels = len(frontiers)
        self.last_frontier_clusters = len(clusters)
        # ToF-derived vertical bounds reject goals that fit the map but violate
        # the vehicle's current floor/ceiling clearance envelope.
        lower = -math.inf if not math.isfinite(self.down) else position[2]-self.down+self.args.vertical_clearance
        upper = math.inf if not math.isfinite(self.up) else position[2]+self.up-self.args.vertical_clearance
        candidates=[]
        valid_clusters=0
        for group in clusters.values():
            if len(group) < self.args.min_frontier_cluster:
                continue
            valid_clusters += 1
            cell=min(group,key=lambda c:sum((c[i]-start[i])**2 for i in range(3)))
            if any(math.dist(cell,old)*self.args.voxel_size<1.0 for old in self.excluded): continue
            candidate_z = self.centre(cell)[2]
            if candidate_z < lower or candidate_z > upper:
                continue
            gain=0
            for dx in range(-2,3):
                for dy in range(-2,3):
                    for dz in range(-2,3):
                        neighbor=(cell[0]+dx,cell[1]+dy,cell[2]+dz)
                        gain += self.within_bounds(neighbor) and self.occupancy.state(neighbor) == UNKNOWN
            distance=math.dist(cell,start)*self.args.voxel_size
            vertical_change=abs(candidate_z-position[2])
            candidates.append((gain/(1.0+distance+3.0*vertical_change),gain,cell))
        self.last_valid_frontier_clusters = valid_clusters
        for _,gain,cell in sorted(candidates,reverse=True)[:48]:
            path=self.astar(safe,start,cell)
            if path:
                self.target=cell; self.path=path; self.path_index=0
                self.target_best_distance = math.dist(start,cell)*self.args.voxel_size
                self.target_progress_at = time.monotonic()
                self.publish_path()
                point=self.centre(cell)
                print(f"3D frontier selected: xyz=({point[0]:.2f},{point[1]:.2f},{point[2]:.2f}) gain={gain} path={len(path)}",flush=True)
                return True
        print(f"No reachable 3D frontier: frontier_voxels={len(frontiers)} clusters={len(clusters)} free={len(free)} occupied={len(occupied)}",flush=True)
        return False

    def local_obstacle(self, velocity_map: np.ndarray) -> float:
        if self.cloud is None or np.linalg.norm(velocity_map)<1e-4: return math.inf
        pose=self.body_pose()
        if pose is None: return math.inf
        message=self.cloud; count=min(message.width*message.height,len(message.data)//message.point_step)
        points=np.ndarray((count,3),dtype=np.float32,buffer=message.data,strides=(message.point_step,4))
        points=points[np.isfinite(points).all(axis=1)]
        ranges=np.linalg.norm(points,axis=1); points=points[(ranges>.45)&(ranges<self.args.sensor_range)]
        direction=pose[1].T@velocity_map
        direction/=max(1e-9,np.linalg.norm(direction))
        forward=points@direction
        lateral=np.linalg.norm(points-forward[:,None]*direction,axis=1)
        # Swept-volume test: the largest body half-extent is required here,
        # otherwise a tabletop below a horizontal path is invisible to a
        # narrow centre-line corridor until the downward ToF is already over it.
        swept_radius=max(self.args.horizontal_radius,self.args.vertical_radius)
        corridor=forward[(forward>0)&(lateral<swept_radius)]
        return float(np.min(corridor)) if len(corridor) else math.inf

    def velocity_to_mavros(self, velocity_map: np.ndarray) -> tuple[float,float,float]:
        try:
            transform=self.tf.lookup_transform("world",self.planning_frame,Time())
            angle=yaw(transform.transform.rotation); c,s=math.cos(angle),math.sin(angle)
            wx=c*velocity_map[0]-s*velocity_map[1]; wy=s*velocity_map[0]+c*velocity_map[1]
        except TransformException:
            wx,wy=velocity_map[:2]
        return (-float(wy),float(wx),float(velocity_map[2]))

    def safe_command(self, desired: np.ndarray) -> tuple[float,float,float,float]:
        obstacle=self.local_obstacle(desired)
        scale=max(0.0,min(1.0,(obstacle-self.args.emergency_distance)/0.7))
        # Start removing horizontal speed before a sudden floor/ceiling step
        # reaches the hard clearance limit (for example when crossing a table).
        if math.isfinite(self.down):
            scale=min(scale,max(0.0,min(1.0,(self.down-self.args.vertical_clearance)/self.args.vertical_slow_margin)))
        if math.isfinite(self.up):
            scale=min(scale,max(0.0,min(1.0,(self.up-self.args.vertical_clearance)/self.args.vertical_slow_margin)))
        desired=desired*scale
        if self.down < self.args.vertical_clearance+self.args.vertical_slow_margin:
            desired[2]=max(0.20,desired[2])
        if self.up < self.args.vertical_clearance+self.args.vertical_slow_margin:
            desired[2]=min(-0.20,desired[2])
        if self.down<self.args.vertical_clearance:
            desired[:2]=0.0; desired[2]=max(0.25,desired[2])
        if self.up<self.args.vertical_clearance:
            desired[:2]=0.0; desired[2]=min(-0.25,desired[2])
        return (*self.velocity_to_mavros(desired),obstacle)

    def path_valid(self, safe: set[Voxel]) -> bool:
        return all(cell in safe for cell in self.path[self.path_index:self.path_index+5])

    def publish_path(self) -> None:
        message=RosPath(); message.header.frame_id=self.planning_frame; message.header.stamp=self.simulation_stamp()
        for cell in self.path:
            xyz=self.centre(cell); pose=PoseStamped(); pose.header=message.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,xyz); pose.pose.orientation.w=1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def publish_markers(self) -> None:
        free,occupied,safe=self.map_sets(); now=self.simulation_stamp(); array=MarkerArray()
        for marker_id,(name,cells,color,size) in enumerate((
            ("frontier",{c for c in safe if is_frontier(c,self.occupancy.states,self.within_bounds,NEIGHBORS_6,self.args.frontier_unknown_neighbors,2)},(0.1,1.0,0.2,0.8),self.args.voxel_size*.35),
        )):
            marker=Marker(); marker.header.frame_id=self.planning_frame; marker.header.stamp=now; marker.ns=name; marker.id=marker_id
            marker.type=Marker.CUBE_LIST; marker.action=Marker.ADD; marker.scale.x=marker.scale.y=marker.scale.z=size
            marker.pose.orientation.w=1.0
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=color
            cells = list(cells)
            if len(cells) > self.args.max_visual_voxels:
                stride = math.ceil(len(cells)/self.args.max_visual_voxels)
                cells = cells[::stride]
            for cell in cells:
                xyz=self.centre(cell); p=Point(); p.x,p.y,p.z=map(float,xyz); marker.points.append(p)
            array.markers.append(marker)
        self.marker_pub.publish(array)
        # PointCloud2 is substantially more reliable and cheaper in RViz than
        # a CUBE_LIST marker containing thousands of occupied voxels.
        cells=list(occupied)
        if len(cells)>self.args.max_visual_voxels:
            cells=cells[::math.ceil(len(cells)/self.args.max_visual_voxels)]
        data=np.empty((len(cells),4),dtype=np.float32)
        if cells:
            data[:,:3]=(np.asarray(cells,dtype=np.float32)+0.5)*self.args.voxel_size
            data[:,3]=np.asarray([self.scores[cell] for cell in cells],dtype=np.float32)
        cloud=PointCloud2();cloud.header.frame_id=self.planning_frame;cloud.header.stamp=now
        cloud.height=1;cloud.width=len(cells);cloud.fields=[PointField(name="x",offset=0,datatype=PointField.FLOAT32,count=1),PointField(name="y",offset=4,datatype=PointField.FLOAT32,count=1),PointField(name="z",offset=8,datatype=PointField.FLOAT32,count=1),PointField(name="intensity",offset=12,datatype=PointField.FLOAT32,count=1)]
        cloud.is_bigendian=False;cloud.point_step=16;cloud.row_step=16*len(cells);cloud.is_dense=True;cloud.data=data.tobytes()
        self.voxel_snapshot_pub.publish(cloud)

    def save_checkpoint(self, output: Path) -> None:
        """Atomically persist the sparse map during long autonomous runs."""
        cells=np.asarray(list(self.scores),dtype=np.int32)
        values=np.asarray([self.scores[tuple(cell)] for cell in cells],dtype=np.int8)
        states=np.asarray([self.occupancy.state(tuple(cell)) for cell in cells],dtype=np.int8)
        temporary=output/"voxel_map_partial.tmp.npz"
        np.savez_compressed(temporary,cells=cells,log_odds=values,states=states,resolution=self.args.voxel_size,frame=self.planning_frame,schema="sparse_quantized_log_odds_v2")
        temporary.replace(output/"voxel_map_partial.npz")
        free,occupied,_=self.map_sets(); known_xy={(x,y) for x,y,_ in free|occupied}
        progress={"evidence_voxels":len(self.scores),"known_voxels":len(free)+len(occupied),"uncertain_voxels":len(self.scores)-len(free)-len(occupied),"occupied_voxels":len(occupied),"free_voxels":len(free),"known_xy_cells":len(known_xy),"frontier_voxels":self.last_frontier_voxels,"frontier_clusters":self.last_frontier_clusters,"valid_frontier_clusters":self.last_valid_frontier_clusters,"state_transitions":self.occupancy.state_transition_count,"direct_occupied_free_flips":self.occupancy.direct_flip_count,"updated_wall_time":time.time()}
        (output/"progress.json").write_text(json.dumps(progress,indent=2)+"\n")

    def run(self, output: Path) -> None:
        output.mkdir(parents=True,exist_ok=True); rows=[]; self.prepare_guided()
        wait=time.monotonic()+30
        while rclpy.ok() and self.body_pose() is None and time.monotonic()<wait:self.spin_once(.1)
        if self.body_pose() is None: raise RuntimeError("Live /slam/odom unavailable")
        self.publish_metadata()
        deadline=math.inf if self.args.max_duration <= 0 else time.monotonic()+self.args.max_duration
        last_plan=0.0; no_frontier=0; last_status=0.0
        last_checkpoint=0.0
        completion_reason="time_limit"; exploration_complete=False
        start=self.position(); home=self.voxel(start); self.breadcrumbs=[home]
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin_once(.01); now=time.monotonic(); updated=self.integrate_cloud(); position=self.position()
            if position is None:
                self.send_velocity(0.0,0.0,0.0)
                continue
            current=self.voxel(position)
            if current!=self.breadcrumbs[-1]: self.breadcrumbs.append(current)
            if self.target is not None and np.linalg.norm(position-self.centre(self.target))<.50:
                self.excluded.append(self.target); self.target=None; self.path=[]
            if self.target is not None:
                target_distance=float(np.linalg.norm(position-self.centre(self.target)))
                if target_distance < self.target_best_distance-0.12:
                    self.target_best_distance=target_distance; self.target_progress_at=now
                elif now-self.target_progress_at > self.args.stuck_timeout:
                    print(f"Frontier progress stalled for {self.args.stuck_timeout:.0f}s; excluding target",flush=True)
                    self.excluded.append(self.target); self.target=None; self.path=[]
                    self.send_velocity(0.0,0.0,0.0)
            if updated and self.path:
                _,_,safe=self.map_sets()
                if not self.path_valid(safe):
                    print("New 3D occupancy invalidated path; stopping and replanning",flush=True)
                    if self.target:self.excluded.append(self.target)
                    self.target=None; self.path=[]; self.send_velocity(0.0,0.0,0.0)
            if self.target is None and now-last_plan>2.0:
                if self.choose_frontier(): no_frontier=0
                else: no_frontier+=1
                last_plan=now
            if no_frontier>=self.args.completion_checks and now-self.last_map_growth_at>=self.args.completion_settle:
                completion_reason="frontier_exhausted"
                exploration_complete=True
                break
            desired=np.zeros(3,dtype=np.float64)
            if self.path:
                while self.path_index<len(self.path)-1 and np.linalg.norm(position-self.centre(self.path[self.path_index]))<.45:
                    self.path_index+=1
                waypoint=self.centre(self.path[min(len(self.path)-1,self.path_index+1)])
                delta=waypoint-position; distance=float(np.linalg.norm(delta))
                desired=delta/max(.01,distance)*min(self.args.speed,.8*distance)
            mx,my,mz,obstacle=self.safe_command(desired); self.send_velocity(mx,my,mz)
            rows.append((now,*position,self.down,self.up,obstacle,mx,my,mz,len(self.scores)))
            if now-last_status>2:
                free,occupied,_=self.map_sets(); observed_xy=len({(x,y) for x,y,_ in free|occupied})
                bounds_area=(self.args.max_x-self.args.min_x)*(self.args.max_y-self.args.min_y)
                coverage=100.0*observed_xy*self.args.voxel_size**2/bounds_area
                print(f"3D explorer: xyz=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) down={self.down:.2f} up={self.up:.2f} obstacle={obstacle:.2f} voxels={len(self.scores)} xy_coverage={coverage:.1f}% frontiers={self.last_frontier_clusters}",flush=True); last_status=now
            if now-self.last_visualize>2.0:self.publish_markers();self.last_visualize=now
            if now-last_checkpoint>30.0:self.save_checkpoint(output);last_checkpoint=now
            time.sleep(.025)
        # Return by the shortest currently known-safe 3D route. Fall back to
        # breadcrumbs only if the sparse map no longer has a connected route.
        position = self.position()
        _,_,safe = self.map_sets()
        direct_home = self.astar(safe, self.voxel(position), home) if position is not None else []
        self.path = direct_home or list(reversed(self.breadcrumbs))
        self.path_index=0; self.publish_path(); end=time.monotonic()+120
        final_position = position
        while rclpy.ok() and time.monotonic()<end:
            self.spin_once(.02); position=self.position()
            if position is None:
                self.send_velocity(0.0,0.0,0.0)
                continue
            final_position = position
            if np.linalg.norm(position-start)<.30: break
            while self.path_index<len(self.path)-1 and np.linalg.norm(position-self.centre(self.path[self.path_index]))<.45:self.path_index+=1
            waypoint = start if self.path_index >= len(self.path)-1 else self.centre(self.path[self.path_index+1])
            delta=waypoint-position; distance=np.linalg.norm(delta)
            command=delta/max(.01,distance)*min(self.args.speed,.8*distance); mx,my,mz,_=self.safe_command(command);self.send_velocity(mx,my,mz);time.sleep(.03)
        self.request_land()
        with (output/"exploration_3d.csv").open("w",newline="") as stream:
            writer=csv.writer(stream);writer.writerow(("wall_time","x","y","z","down","up","obstacle","mavros_vx","mavros_vy","mavros_vz","voxel_count"));writer.writerows(rows)
        cells=np.asarray(list(self.scores),dtype=np.int32); values=np.asarray([self.scores[tuple(c)] for c in cells],dtype=np.int8); states=np.asarray([self.occupancy.state(tuple(c)) for c in cells],dtype=np.int8)
        np.savez_compressed(output/"voxel_map.npz",cells=cells,log_odds=values,states=states,resolution=self.args.voxel_size,frame=self.planning_frame,schema="sparse_quantized_log_odds_v2")
        cfg=self.occupancy.config
        (output/"voxel_map_metadata.json").write_text(json.dumps({"schema":"sparse_quantized_log_odds_v2","frame":self.planning_frame,"resolution_m":self.args.voxel_size,"score_range":[cfg.minimum,cfg.maximum],"occupied_enter_exit":[cfg.occupied_enter,cfg.occupied_exit],"free_enter_exit":[cfg.free_enter,cfg.free_exit],"robot_id":"drone_1","bounds_m":{"min":[self.args.min_x,self.args.min_y,self.args.min_z],"max":[self.args.max_x,self.args.max_y,self.args.max_z]}},indent=2)+"\n")
        free,occupied,_=self.map_sets(); observed_xy=len({(x,y) for x,y,_ in free|occupied})
        bounds_xy_area=(self.args.max_x-self.args.min_x)*(self.args.max_y-self.args.min_y)
        summary={"samples":len(rows),"completion_reason":completion_reason,"exploration_complete":exploration_complete,"evidence_voxels":len(self.scores),"known_voxels":len(free)+len(occupied),"uncertain_voxels":len(self.scores)-len(free)-len(occupied),"occupied_voxels":len(occupied),"free_voxels":len(free),"known_volume_m3":(len(free)+len(occupied))*self.args.voxel_size**3,"observed_xy_area_m2":observed_xy*self.args.voxel_size**2,"bounded_xy_coverage":min(1.0,observed_xy*self.args.voxel_size**2/bounds_xy_area),"remaining_frontier_voxels":self.last_frontier_voxels,"remaining_frontier_clusters":self.last_frontier_clusters,"remaining_valid_frontier_clusters":self.last_valid_frontier_clusters,"state_transitions":self.occupancy.state_transition_count,"direct_occupied_free_flips":self.occupancy.direct_flip_count,"excluded_frontiers":len(self.excluded),"returned_home_error_m":float(np.linalg.norm(final_position-start)) if final_position is not None else None,"minimum_down_m":min((r[4] for r in rows),default=None),"minimum_up_m":min((r[5] for r in rows),default=None)}
        (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n");print(json.dumps(summary,indent=2),flush=True)


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument("--output-dir",type=Path,required=True);parser.add_argument("--max-duration",type=float,default=0,help="0 explores until frontier exhaustion");parser.add_argument("--speed",type=float,default=.40);parser.add_argument("--voxel-size",type=float,default=.30);parser.add_argument("--map-period",type=float,default=.75);parser.add_argument("--max-rays",type=int,default=900);parser.add_argument("--sensor-range",type=float,default=12.0);parser.add_argument("--horizontal-radius",type=float,default=.35);parser.add_argument("--vertical-radius",type=float,default=.50);parser.add_argument("--vertical-clearance",type=float,default=.60);parser.add_argument("--vertical-slow-margin",type=float,default=.30);parser.add_argument("--emergency-distance",type=float,default=.60);parser.add_argument("--max-astar-voxels",type=int,default=150000);parser.add_argument("--max-visual-voxels",type=int,default=20000);parser.add_argument("--frontier-unknown-neighbors",type=int,default=2);parser.add_argument("--min-frontier-cluster",type=int,default=2);parser.add_argument("--completion-checks",type=int,default=8);parser.add_argument("--completion-settle",type=float,default=15.0);parser.add_argument("--stuck-timeout",type=float,default=15.0);parser.add_argument("--min-x",type=float,default=-49.0);parser.add_argument("--max-x",type=float,default=26.5);parser.add_argument("--min-y",type=float,default=-4.5);parser.add_argument("--max-y",type=float,default=37.0);parser.add_argument("--min-z",type=float,default=-.20);parser.add_argument("--max-z",type=float,default=3.20);args=parser.parse_args()
    rclpy.init();node=VoxelExplorer(args)
    try:node.run(args.output_dir)
    finally:
        if node.state.armed:node.request_land()
        node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
