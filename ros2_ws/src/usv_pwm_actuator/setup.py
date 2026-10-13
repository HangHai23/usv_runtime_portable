from glob import glob
import os

from setuptools import find_packages, setup


package_name = "usv_pwm_actuator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="hai",
    maintainer_email="hai@example.com",
    description="Dual-channel I2C/UART PWM actuator for the USV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pwm_actuator_node = usv_pwm_actuator.pwm_actuator_node:main",
        ],
    },
)
