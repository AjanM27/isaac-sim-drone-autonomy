#!/usr/bin/env python3
"""Offline regression tests for stable, reversible voxel occupancy."""

import unittest

from voxel_occupancy import FREE, OCCUPIED, UNKNOWN, ProbabilisticVoxelMap, is_frontier, ray_voxels


class ProbabilisticVoxelMapTest(unittest.TestCase):
    def test_single_noisy_hit_stays_unknown(self):
        occupancy = ProbabilisticVoxelMap()
        occupancy.update([], [(1, 2, 3)])
        self.assertEqual(occupancy.state((1, 2, 3)), UNKNOWN)

    def test_repeated_hits_create_stable_occupied_cell(self):
        occupancy = ProbabilisticVoxelMap()
        occupancy.update([], [(1, 2, 3)])
        occupancy.update([], [(1, 2, 3)])
        self.assertEqual(occupancy.state((1, 2, 3)), OCCUPIED)
        occupancy.update([(1, 2, 3)], [])
        self.assertEqual(occupancy.state((1, 2, 3)), OCCUPIED)

    def test_repeated_clear_rays_correct_occupied_to_free(self):
        cell = (1, 2, 3)
        occupancy = ProbabilisticVoxelMap()
        for _ in range(4):
            occupancy.update([], [cell])
        for _ in range(12):
            occupancy.update([cell], [])
        self.assertEqual(occupancy.state(cell), FREE)
        self.assertEqual(occupancy.direct_flip_count, 0)

    def test_three_clear_observations_are_needed_for_free(self):
        cell = (4, 5, 6)
        occupancy = ProbabilisticVoxelMap()
        occupancy.update([cell], [])
        occupancy.update([cell], [])
        self.assertEqual(occupancy.state(cell), UNKNOWN)
        occupancy.update([cell], [])
        self.assertEqual(occupancy.state(cell), FREE)

    def test_hit_wins_when_cell_is_also_traversed(self):
        cell = (7, 8, 9)
        occupancy = ProbabilisticVoxelMap()
        occupancy.update([cell], [cell])
        self.assertEqual(occupancy.evidence[cell], 9)


class FrontierTest(unittest.TestCase):
    def test_only_stable_free_cells_are_frontiers(self):
        origin = (0, 0, 0)
        neighbors = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        states = {origin: FREE, (1, 0, 0): FREE, (-1, 0, 0): FREE}
        bounded = lambda cell: all(-2 <= value <= 2 for value in cell)
        self.assertTrue(is_frontier(origin, states, bounded, neighbors))
        states[origin] = OCCUPIED
        self.assertFalse(is_frontier(origin, states, bounded, neighbors))

    def test_uncertain_neighbors_remain_unknown(self):
        origin = (0, 0, 0)
        neighbors = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        states = {origin: FREE, (1, 0, 0): FREE, (-1, 0, 0): FREE}
        self.assertTrue(is_frontier(origin, states, lambda _: True, neighbors))


class RayTraversalTest(unittest.TestCase):
    def test_axis_aligned_ray_excludes_hit_cell(self):
        self.assertEqual(
            ray_voxels((0.1, 0.1, 0.1), (1.1, 0.1, 0.1), 0.3),
            [(0, 0, 0), (1, 0, 0), (2, 0, 0)],
        )

    def test_diagonal_ray_is_deterministic_and_bounded(self):
        cells = ray_voxels((-0.1, 0.1, 0.1), (0.95, 0.95, 0.95), 0.3)
        self.assertEqual(cells[0], (-1, 0, 0))
        self.assertNotIn((3, 3, 3), cells)
        self.assertEqual(len(cells), len(set(cells)))


if __name__ == "__main__":
    unittest.main()
