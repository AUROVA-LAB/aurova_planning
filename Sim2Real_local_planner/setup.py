from setuptools import setup
import os
from glob import glob

package_name = 'sim2real_local_planner'
setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    packages=[package_name],
    maintainer='Alejandro Olivas',
    maintainer_email="alejandro.olivas@ua.es",
    description='End-to-end local planner trained in highly realistic simulation and transfered to reality.',
    license='LGPL',
    entry_points={
        'console_scripts': [
            'agent_ros = sim2real_local_planner.agent_ros:main',
        ],
    },
)
