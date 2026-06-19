from setuptools import find_packages, setup
import setuptools.command.install
import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))
package_name = "restaurant"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        import ament_virtualenv.install

        ament_virtualenv.install.install_venv(
            scripts_base=self.install_scripts,
            install_base=self.install_base,
            package_name=package_name,
            python_version="3",
            source_dir=_here,
        )


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="illia",
    maintainer_email="you@example.com",
    description="Restaurant task",
    license="MIT",
    extras_require={"test": ["pytest"]},
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "sm = restaurant.state_machine:main",
            "survey = restaurant.states.survey:main",
            "detect_wave = restaurant.states.detect_wave:main",
            "take_order_sm = restaurant.states.take_order_sm:main", 
            "get_order_from_bar = restaurant.states.get_order_from_bar:main",
            ],
    },
)   
