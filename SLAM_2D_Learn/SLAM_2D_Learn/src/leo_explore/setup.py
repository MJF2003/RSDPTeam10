from setuptools import setup
from glob import glob
import os

package_name = 'leo_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yht',
    maintainer_email='todo@todo.com',
    description='Simple autonomous wandering for Leo rover',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wander = leo_explore.wander:main',
            'explore_action_server = leo_explore.explore_action_server:main',
            'frontier_explorer = leo_explore.frontier_explorer:main',
        ],
    },
)
