from glob import glob
from setuptools import find_packages, setup


package_name = "usv_imu_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hai",
    maintainer_email="hai@example.com",
    description="Yahboom serial IMU driver and visualization for the USV.",
    license="MIT",
    entry_points={"console_scripts": ["imu_node = usv_imu_driver.imu_node:main"]},
)
