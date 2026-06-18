from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))

package_name = "HRI"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            scripts_base=self.install_scripts,
            install_base=self.install_base,
            package_name=package_name,
            python_version="3",
            source_dir=_here,
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
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
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
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "seat_guest = HRI.states.seat_guest:main",
            "sm = HRI.state_machine:main",
            "start_sm = HRI.states.start_door_sm:main",
            "recognise = HRI.states.recognise:main",
        ],
    },
)
