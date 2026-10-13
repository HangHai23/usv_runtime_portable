from glob import glob
from setuptools import find_packages, setup


package_name = "usv_camera_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hai",
    maintainer_email="hai@example.com",
    description="Reconnectable USB camera driver for the USV vision pipeline.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = usv_camera_driver.camera_node:main",
        ]
    },
)
