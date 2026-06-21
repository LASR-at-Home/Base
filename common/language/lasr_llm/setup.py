import os
from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

_here = os.path.dirname(os.path.abspath(__file__))


package_name = "lasr_llm"


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
    packages=find_packages(exclude=["test"]),
    cmdclass={"install": InstallCommand},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yara",
    maintainer_email="yara.alkhelaiwi@kcl.ac.uk",
    description="The lasr_llm package",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "receptionist_service = lasr_llm.nodes.receptionist_service:main",
            "hri_task_service = lasr_llm.nodes.hri_task_service:main",
            "storing_groceries_service = lasr_llm.nodes.storing_groceries_service:main",
            "restaurant_service = lasr_llm.nodes.restaurant_service:main",
        ],
    },
)
