from setuptools import setup

package_name = 'uav_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandhiya',
    maintainer_email='sandhiya@todo.todo',
    description='GPS-denied UAV autonomy and navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'state_bridge = uav_autonomy.state_bridge:main',
            'flight_controller = uav_autonomy.flight_controller:main',
        ],
    },
)
