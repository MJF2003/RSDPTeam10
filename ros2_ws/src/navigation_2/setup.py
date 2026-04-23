from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/behavior_trees'), glob('config/behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student49',
    maintainer_email='student49@todo.todo',
    description='Navigation Package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'navigation_service_node = navigation_2.navigation_service_node:main',],
    },
)