from glob import glob
from setuptools import find_packages, setup


package_name = 'stemm_cartographer_exploration'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/deploy', glob('deploy/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='Independent Cartographer-based STEMM autonomous exploration stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stemm_cartographer_preflight = '
            'stemm_cartographer_exploration.preflight_guard:main',
            'stemm_cartographer_odom_session_guard = '
            'stemm_cartographer_exploration.odom_session_guard:main',
            'stemm_cartographer_recharge_stop_guard = '
            'stemm_cartographer_exploration.recharge_stop_guard:main',
            'stemm_cartographer_map_service_adapter = '
            'stemm_cartographer_exploration.map_service_adapter:main',
            'stemm_cartographer_nav2_ready_guard = '
            'stemm_cartographer_exploration.nav2_ready_guard:main',
            'stemm_cartographer_nav2_manager = '
            'stemm_cartographer_exploration.cartographer_manager:main',
            'stemm_cartographer_state_saver = '
            'stemm_cartographer_exploration.state_saver:main',
            'stemm_cartographer_session_runner = '
            'stemm_cartographer_exploration.session_runner:main',
        ],
    },
)
