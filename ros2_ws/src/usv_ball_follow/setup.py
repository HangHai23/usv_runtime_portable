from glob import glob
from setuptools import find_packages, setup


package_name = "usv_ball_follow"

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
    description="IMU-assisted target estimation and ball-following control for the USV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "target_state_estimator = usv_ball_follow.target_state_estimator:main",
            "ball_follow_controller = usv_ball_follow.ball_follow_controller:main",
            "telemetry_recorder = usv_ball_follow.telemetry_recorder:main",
        ]
    },
)
