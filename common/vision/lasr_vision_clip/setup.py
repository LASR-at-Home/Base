from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install


package_name = "lasr_vision_clip"

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
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaliyah",
    maintainer_email="aaliyah.merchant@kcl.ac.uk",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "test_vqa = lasr_vision_clip.nodes.test_vqa:main",
            "vqa = lasr_vision_clip.nodes.vqa:main",
        ]
    },
)
