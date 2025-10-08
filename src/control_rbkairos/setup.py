from setuptools import setup
from glob import glob
import os

package_name = 'control_rbkairos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ชื่อคุณ',
    maintainer_email='อีเมลคุณ',
    description='description',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = control_rbkairos.move_robot:main',
            'robot_movement = control_rbkairos.scripts.robot_movement:main',
        ],
    },
)
