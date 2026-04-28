from setuptools import find_packages, setup

package_name = 'leo_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student49',
    maintainer_email='student49@todo.todo',
    description='Frontier exploration action server for the rover.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'explore_action_server = '
            'leo_explore.explore_action_server:main',
        ],
    },
)
