from glob import glob
from setuptools import find_packages, setup


package_name = "isaac_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AjanM27",
    maintainer_email="AjanM27@users.noreply.github.com",
    description="RTAB-Map 3D LiDAR SLAM integration for Isaac Sim.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_health_check = isaac_slam.health_check:main",
        ],
    },
)
