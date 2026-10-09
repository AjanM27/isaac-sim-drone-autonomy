# Security policy

Please report a suspected vulnerability privately through GitHub's security
advisory feature for this repository. Do not publish credentials, local network
details or a working exploit in a public issue.

This repository starts host-network Docker containers and accepts UDP/MAVLink
traffic intended for local simulation. It is not hardened for an untrusted
network, public internet exposure or real-aircraft deployment. Keep the host
firewall enabled and do not forward the SITL, DDS or MAVLink ports externally.

For vulnerabilities in NVIDIA Isaac Sim, ArduPilot, ROS 2, MAVROS, RTAB-Map or
Docker, use the upstream project's security process.
