from setuptools import setup, find_packages

package_name = 'drone_pkg'

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
    description='Camera streaming, environmental sensors, and mission reception for the UAV.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = drone_pkg.camera_node:main',
            'env_sensor_node = drone_pkg.env_sensor_node:main',
            'mission_receiver_node = drone_pkg.mission_receiver_node:main',
        ],
    },
)
