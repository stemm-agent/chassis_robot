import os
from glob import glob

from setuptools import setup

package_name = 'cloud_video_pusher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='robot@example.com',
    description='ROS2 image topic to RTMP cloud video pusher demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtmp_image_pusher = cloud_video_pusher.rtmp_image_pusher:main',
            'local_hls_image_server = cloud_video_pusher.local_hls_image_server:main',
        ],
    },
)