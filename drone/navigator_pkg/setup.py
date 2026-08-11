from setuptools import setup, find_packages

package_name = 'navigator_pkg'

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
    description='Mission upload and execution via MAVROS.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigator_node = navigator_pkg.navigator_node:main',
        ],
    },
)
