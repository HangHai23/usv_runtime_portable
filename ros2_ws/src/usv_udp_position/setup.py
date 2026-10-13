from glob import glob
from setuptools import setup

setup(
    name='usv_udp_position', version='0.1.0', packages=['usv_udp_position'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/usv_udp_position']),
        ('share/usv_udp_position', ['package.xml']),
        ('share/usv_udp_position/config', glob('config/*.yaml')),
        ('share/usv_udp_position/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='hai', maintainer_email='hai@example.com', license='MIT',
    description='Receive global Cartesian USV positions over UDP.',
    entry_points={'console_scripts': ['udp_position_node = usv_udp_position.node:main']},
)
