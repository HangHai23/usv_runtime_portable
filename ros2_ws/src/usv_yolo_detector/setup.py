from glob import glob
from setuptools import find_packages, setup


package_name = "usv_yolo_detector"

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
    description="Portable ONNX CPU and optional TensorRT YOLO target detector.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_detector_node = usv_yolo_detector.yolo_detector_node:main",
        ]
    },
)
