from setuptools import find_packages, setup

package_name = 'uav_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'rescue_mission = uav_autonomy.rescue_mission:main',
            'flight_controller = uav_autonomy.flight_controller:main',
            'state_bridge = uav_autonomy.state_bridge:main',
        'vlm_detector = uav_autonomy.vlm_detector:main',
        ],
    },
)
