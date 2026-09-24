from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))

package_name = "GPSR"


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
    maintainer="Michele Brienza",
    maintainer_email="michelebrienza1997@gmail.com",
    description="GPSR autonomous behaviour task",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "sm = GPSR.state_machine:main",
            "dispatch_skill = GPSR.states.dispatch_skill:main",
            "service = GPSR.service:main",
        ],
    },
)
