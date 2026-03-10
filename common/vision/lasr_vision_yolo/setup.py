from setuptools import find_packages, setup
import os
from glob import glob
import setuptools.command.install
import ament_virtualenv.install

package_name = "lasr_vision_yolo"

class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version='3'
        )
        return

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maayan",
    maintainer_email="maayan.armony@gmail.com",
    description="YOLO object detection service",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={'install': InstallCommand},
    entry_points={
        "console_scripts": [
            "yolo_service_node = lasr_vision_yolo.service:main",
            "yolo_node = src.lasr_vision_yolo.yolo:main",
            "relay = lasr_vision_yolo.relay_test:main",
        ],
    },
)
