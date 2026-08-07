from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lane_detector'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # models (recursive)
        *[
            (os.path.join('share', package_name, os.path.dirname(f)),
             [f])
            for f in glob('models/**/*', recursive=True)
            if os.path.isfile(f)
        ],
        # urdf
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aniasse',
    maintainer_email='ibrahimaniasse@github.com',
    description='ROS 2 lane-following for TurtleBot3 on a ramp world',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_lane   = lane_detector.detect_lane:main',
            'control_lane  = lane_detector.control_lane:main',
            'control_blind = lane_detector.control_blind:main',
            'test_ramp_climb = lane_detector.test_ramp_climb:main',
        ],
    },
)
