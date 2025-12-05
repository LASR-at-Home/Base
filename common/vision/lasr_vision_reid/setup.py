from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

package_name = "lasr_vision_reid"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            scripts_base=self.install_scripts,
            install_base=self.install_base,
            package_name=package_name,
            python_version='3'
        )
        # instead of self.install_base we may also use:
        # self.config_vars['platbase'] or self.config_vars['base']
        # Exchange the python_version with '3' if your package uses Python3.
        return

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fadi-mostefai",
    maintainer_email="fadimostefai@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["service = lasr_vision_reid.service:main",
                            "relay_3d = lasr_vision_reid.relay_3d:main",
                            "add_face = lasr_vision_reid.add_face:main"],
    },
    cmdclass={
        'install': InstallCommand
    }
)
