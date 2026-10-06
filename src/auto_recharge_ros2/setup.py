from setuptools import setup

package_name = 'auto_recharge_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'robot_info.yaml',
                                   'Charger_Position.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "auto_recharge = auto_recharge_ros2.auto_recharger:main",
            "navigate_to_charger = auto_recharge_ros2.navigate_to_charger:main",
            "cancel_charge = auto_recharge_ros2.cancel_charge:main",
            "low_battery_auto_recharge = auto_recharge_ros2.low_battery_auto_recharge:main",
            "recharge_manager = auto_recharge_ros2.recharge_manager:main",
        ],
    },
)
