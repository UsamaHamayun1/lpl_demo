import os
from glob import glob
from setuptools import setup

package_name = 'lpl_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- ADDED LINES START ---
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # ros_gz bridge / Nav2 config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # --- ADDED LINES END ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usama',
    maintainer_email='usama.humayun1@gmail.com',
    description='LPL Demo Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lpl_manager = lpl_demo.lpl_manager:main',
            'lpl_manager_formula = lpl_demo.lpl_manager_formula:main',
            'human_authorize = lpl_demo.human_authorize:main',
            'lpl_dashboard = lpl_demo.lpl_dashboard:main',
            'lpl_actordirector = lpl_demo.lpl_actordirector:main',
            'lpl_manager_gauntlet = lpl_demo.lpl_manager_gauntlet:main',
            'lpl_manager_guard = lpl_demo.lpl_manager_guard:main',
            'lpl_nav2_demo = lpl_demo.lpl_nav2_demo:main',
            'gz_world_state = lpl_demo.gz_world_state:main',
            
            ],
    },
)   