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
            'vlm_perception = uav_autonomy.vlm_perception:main',
        ],
    },
)
