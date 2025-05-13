import os
from glob import glob

from setuptools import setup

package_name = 'gymbrot_point_nav_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heura',
    maintainer_email='frogadier100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = gymbrot_point_nav_system.initial_pose_pub:main',  # añadir
            'client_navigate_to_pose = gymbrot_point_nav_system.client_navigate_to_pose:main'

        ],
    },
)
