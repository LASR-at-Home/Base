from setuptools import setup
import os
from glob import glob
import setuptools.command.install
import ament_virtualenv.install

package_name = "lasr_manipulation_actions"

_here = os.path.dirname(os.path.abspath(__file__))


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version="3",
            source_dir=_here,
        )
        return


setup(
    name=package_name,
    version="0.0.0",
    packages=["lasr_manipulation_actions"],
    package_dir={"lasr_manipulation_actions": "manipulation_actions"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michele Brienza",
    maintainer_email="michelebrienza1997@gmail.com",
    description="Grasping pipeline: PlayMotion2 pregrasp + arm joint sweep + approach",
    license="MIT",
    extras_require={"test": ["pytest"]},
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "manipulation_actions_node = lasr_manipulation_actions.node:main",
            "grasp_gui = lasr_manipulation_actions.gui:main",
            "arm_cli = lasr_manipulation_actions.arm_cli:main",
        ],
    },
)
