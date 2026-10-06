"""Packaging metadata for the ROS 2 autonomy nodes."""

from setuptools import find_packages, setup

package_name = 'uav_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/p2_perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='UAV Autonomy Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = uav_autonomy.flight_controller:main',
            'state_bridge = uav_autonomy.state_bridge:main',
            'debris_perception = uav_autonomy.debris_perception:main',
            'debris_depth_detector = '
            'uav_autonomy.debris_depth_detector:main',
            'debris_tracker = uav_autonomy.debris_tracker:main',
            'debris_predictor = uav_autonomy.debris_predictor:main',
            'debris_risk = uav_autonomy.debris_risk_node:main',
            'debris_avoidance = uav_autonomy.debris_avoidance_node:main',
            # Phase 7 node. The earlier survivor_perception.py,
            # vlm_perception.py and vlm_detector.py are kept in the
            # package, unreferenced (docs/survivor_perception.md).
            'survivor_perception = '
            'uav_autonomy.survivor_perception_node:main',
            'survivor_localization = '
            'uav_autonomy.survivor_localization_node:main',
            'continuous_debris_spawner = '
            'uav_autonomy.continuous_debris_spawner:main',
        ],
    },
)
