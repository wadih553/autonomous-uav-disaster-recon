from glob import glob
import os
from setuptools import setup

package_name = "uav_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wadih Dahrouge",
    maintainer_email="wadihdahrouge1@gmail.com",
    description="Launch configuration for the autonomous disaster-reconnaissance UAV.",
    license="MIT",
)
