from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'receptionist'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),  # Add the config directory
            glob(os.path.join("config", "*.yaml")),  # Include all YAML files in config
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maayan",
    maintainer_email="maayan.armony@gmail.com",
    description="REceptionist task",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "receptionist_sm = receptionist.main:main"

        ],
    },
)
