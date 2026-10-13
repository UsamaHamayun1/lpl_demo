import os
from glob import glob

from setuptools import setup

package_name = 'lpl2_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usama',
    maintainer_email='usama.humayun1@gmail.com',
    description='LPL 2.0 Nav2 configuration and plain mission runner',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = lpl2_nav.mission:main',
        ],
    },
)
