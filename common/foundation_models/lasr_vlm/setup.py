from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

package_name = "lasr_vlm"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version="3",
        )
        return


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    cmdclass={"install": InstallCommand},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maayan",
    maintainer_email="maayan.armony@gmail.com",
    description="Package for running inference with large vision-language models (VLMs) in LASR",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
