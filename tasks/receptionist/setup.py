from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install
import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))

package_name = "receptionist"


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
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maayan",
    maintainer_email="maayan.armony@gmail.com",
    description="Receptionist task",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            "llm_test = receptionist.states.test_llm:main",
            "string_test = receptionist.states.test_string:main",
            "llm_and_sentence_test = receptionist.states.test_llm_and_sentence:main",
        ]
    },
)
