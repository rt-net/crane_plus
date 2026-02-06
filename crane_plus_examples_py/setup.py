from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'crane_plus_examples_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]')),
        ),
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
            'aruco_detection = crane_plus_examples_py.aruco_detectio:main',
            'color_detection = crane_plus_examples_py.color_detection:main',
            'gripper_control = crane_plus_examples_py.gripper_control:main',
            'joint_values = crane_plus_examples_py.joint_values:main',
            'pick_and_place_tf = crane_plus_examples_py.pick_and_place_tf:main',
            'pick_and_place = crane_plus_examples_py.pick_and_place:main',
            'pose_groupstate= crane_plus_examples_py.pose_groupstate:main',
            'utils = crane_plus_examples_py.utils:main',
        ],
    },
)
