from glob import glob
from setuptools import setup

setup(
    name='usv_mission_manager', version='0.1.0', packages=['usv_mission_manager'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/usv_mission_manager']),
                ('share/usv_mission_manager', ['package.xml']),
                ('share/usv_mission_manager/config', glob('config/*.yaml')),
                ('share/usv_mission_manager/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='hai',
    maintainer_email='hai@example.com', description='Ball acquisition and goal delivery mission',
    license='MIT', entry_points={'console_scripts': ['mission_manager = usv_mission_manager.node:main']})
