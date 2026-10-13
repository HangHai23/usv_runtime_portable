from glob import glob
import os

from setuptools import find_packages, setup


package_name = "usv_web_monitor"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools", "aiohttp"],
    zip_safe=True,
    maintainer="hai",
    maintainer_email="hai@example.com",
    description="Read-only LAN dashboard for USV ROS 2 telemetry.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_monitor = usv_web_monitor.server:main",
        ],
    },
)
