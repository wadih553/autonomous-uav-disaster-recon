from setuptools import setup, find_packages

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wadih Dahrouge',
    maintainer_email='wadihdahrouge1@gmail.com',
    description='360-degree LiDAR obstacle detection and avoidance.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
