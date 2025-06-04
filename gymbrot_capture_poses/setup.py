from setuptools import setup
import os
from glob import glob

package_name = 'gymbrot_capture_poses'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resources'), glob('gymbrot_capture_poses/resources/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcos04',
    maintainer_email='mmaryus1@epsg.upv.es',
    description='Captura y comparación de poses humanas',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'captura_pose = gymbrot_capture_poses.pose_feedback_node:main',
            'publicar_imagen = gymbrot_capture_poses.image_publisher:main',
        ],
    },
)
