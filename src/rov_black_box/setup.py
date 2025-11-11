from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rov_black_box'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),        
        (os.path.join('share', package_name, 'config'), glob('config/*.config.yaml')),
        (os.path.join('share', package_name, 'lib'), glob('*.py')),  
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anastasiia',
    maintainer_email='frolovaanastasiia29@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_node = rov_black_box.joystick_node:main',
            'video_node = rov_black_box.video_node:main',
            'track_box_node = rov_black_box.track_box_node:main',
            'post_mission_node = rov_black_box.post_mission_node:main',

        ],
    },
)
