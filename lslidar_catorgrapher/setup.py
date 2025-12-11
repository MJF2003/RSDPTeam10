from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lslidar_catorgrapher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nzh',
    maintainer_email='nzh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "node_read_map_pos = lslidar_catorgrapher.read_map_pos:main",
            "node_check_block = lslidar_catorgrapher.check_block:main"
        ],
    },
)
