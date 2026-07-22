from glob import glob
from setuptools import setup

package_name = 'stemm_nav2_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='STEMM 2.1 autonomous mapping and navigation integration manager.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stemm_nav2_manager = stemm_nav2_manager.manager:main',
            'mapping_odom_reset = stemm_nav2_manager.mapping_odom_reset:main',
            'mapping_tf_guard = stemm_nav2_manager.mapping_tf_guard:main',
        ],
    },
)
