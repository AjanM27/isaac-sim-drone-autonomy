# Third-party notices

The Apache-2.0 license in this repository applies to project-authored code and
to upstream files already distributed under that license. It does not replace
the licenses of external software or downloaded content.

| Component | Source | License/terms |
|---|---|---|
| NVIDIA Isaac Sim | https://developer.nvidia.com/isaac/sim | NVIDIA Omniverse EULA and package notices |
| NVIDIA Isaac Sim assets | NVIDIA public content storage URLs in `asset_manifest.json` | asset-specific NVIDIA/partner terms |
| Hospital environment | downloaded with its `LICENSE` file | Creative Commons Attribution 4.0 International |
| ArduPilot | https://github.com/ArduPilot/ardupilot | GPL-3.0 |
| ROS 2 | https://github.com/ros2 | primarily Apache-2.0/BSD; package-specific |
| MAVROS | https://github.com/mavlink/mavros | GPL-3.0 |
| RTAB-Map ROS | https://github.com/introlab/rtabmap_ros | BSD-3-Clause |
| OctoMap | https://github.com/OctoMap/octomap | BSD-3-Clause |
| Docker Engine | https://github.com/moby/moby | Apache-2.0 |

Third-party binary assets are not committed to the public repository. The
versioned manifest records their original public source URLs and checksums;
each user downloads them directly from the provider. The custom `iris.usd`
vehicle supplied with this project is distributed for use with this simulator.

Hospital attribution: NVIDIA Isaac Sim Hospital environment, licensed under
CC BY 4.0. A verbatim license copy is downloaded beside the environment as
`assets/isaac/6.0/Isaac/Environments/Hospital/LICENSE`.

Container builds install upstream packages and preserve their license metadata.
Review the package/image notices before redistributing a built image.
