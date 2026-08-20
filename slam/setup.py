import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'gps_denied_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['gps_denied_nav', 'gps_denied_nav.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member 4 (GPS-Denied Navigation Engineer)',
    maintainer_email='member4@drone-team.org',
    description='GPS-Denied Navigation Suite for UAV Localization, SLAM, EKF Fusion, and MAVROS Bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vo_node = gps_denied_nav.vo.vo_node:main',
            'ekf_fusion_node = gps_denied_nav.fusion.ekf_fusion_node:main',
            'tf_broadcaster = gps_denied_nav.bridge.tf_broadcaster:main',
            'mavros_vision_bridge = gps_denied_nav.bridge.mavros_vision_bridge:main',
            'occupancy_grid_node = gps_denied_nav.mapping.occupancy_grid_node:main',
            'diagnostics_node = gps_denied_nav.eval.diagnostics_node:main',
            'synthetic_data_generator = gps_denied_nav.eval.synthetic_data_generator:main',
            'trajectory_evaluator = gps_denied_nav.eval.trajectory_evaluator:main',
        ],
    },
)
