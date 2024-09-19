import os  
from glob import glob
from setuptools import find_packages, setup

package_name = 'crane_plus_examples_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = package_name.aruco_detectio:main',
            'color_detection = package_name.color_detection:main',
            'gripper_control = package_name.gripper_control:main',
            'joint_values = package_name.joint_values:main',
            'pick_and_place_tf = package_name.pick_and_place_tf:main',
            'pick_and_place = package_name.pick_and_place:main',
            'pose_groupstate= package_name.pose_groupstate:main',
            'utils = package_name.utils:main',
        ],
    },
)
