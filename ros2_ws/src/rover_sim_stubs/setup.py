from setuptools import find_packages, setup

package_name = 'rover_sim_stubs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='laptop20',
    maintainer_email='alexinch96@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_stub = rover_sim_stubs.vision_stub:main',
            'navigation_stub = rover_sim_stubs.navigation_stub:main',
            'manipulation_stub = rover_sim_stubs.manipulation_stub:main',
            'nav_debug_overlay = rover_sim_stubs.nav_debug_overlay:main',
        ],
    },
)
