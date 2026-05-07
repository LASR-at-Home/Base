from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

import os
from glob import glob

package_name = "HRI"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            scripts_base=self.install_scripts,
            install_base=self.install_base,
            package_name=package_name,
            python_version="3",
        )
        return


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yara",
    maintainer_email="yaralkhelaiwi@gmail.com",
    description="HRI task",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "seat_guest = HRI.states.seat_guest:main",
            "process_detection_t = HRI.states.process_detection_t:main",  # TODO: DELETE AFTER TESTING
        ],
    },
)
