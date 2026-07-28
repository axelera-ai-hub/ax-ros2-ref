# (C) Copyright Axelera AI 2026
# All Rights Reserved
from setuptools import find_packages, setup

package_name = 'metis_pose_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pose_inference.launch.py',
            'launch/wrist_tracking_test.launch.py',
            'launch/wrist_tracking_full.launch.py',
            'launch/duck_hand_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nihal Suri',
    maintainer_email='nihal.suri@axelera.ai',
    description='BRIO pose inference → wrist TF broadcaster using Axelera voyager-sdk.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_inference_node = metis_pose_inference.pose_inference_node:main',
            'wrist_tracker_node = metis_pose_inference.wrist_tracker_node:main',
        ],
    },
)
