import os
from glob import glob

from setuptools import setup

package_name = 'lpl2_sim'


def share(subdir, pattern='*'):
    return (os.path.join('share', package_name, subdir), glob(os.path.join(subdir, pattern)))


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        share('launch', '*.launch.py'),
        share('config', '*.yaml'),
        share('config/scenarios', '*.yaml'),
        share('worlds', '*.sdf'),
        share('maps'),
        share('models/robot_r'),
        share('urdf', '*.urdf'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usama',
    maintainer_email='usama.humayun1@gmail.com',
    description='LPL 2.0 simulation: corridor world, scripted world events, Robot 2, cameras',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_state = lpl2_sim.world_state:main',
            'robot2_driver = lpl2_sim.robot2_driver:main',
            'scenario_director = lpl2_sim.scenario_director:main',
            'recorder = lpl2_sim.recorder:main',
            'truth_localization = lpl2_sim.truth_localization:main',
        ],
    },
)
