import os
from glob import glob

from setuptools import setup

package_name = 'lpl2_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usama',
    maintainer_email='usama.humayun1@gmail.com',
    description='LPL 2.0 mission control dashboard and mock publishers',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = lpl2_ui.dashboard:main',
            'mock_lpl = lpl2_ui.mock_lpl:main',
            'mock_follower = lpl2_ui.mock_follower:main',
        ],
    },
)
