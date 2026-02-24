#!/usr/bin/env python3

from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

package_name = "skills"
# setup_args = generate_distutils_setup(packages=["lasr_skills"], package_dir={"": "src"})


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version="3",
        )

        # If you get a 'scripts_base' error uncomment the line above.
        return


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    # packages=["lasr_skills"],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maayan Armony",
    maintainer_email="maayan.armony@gmail.com",
    description="The skills package",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={"install": InstallCommand},
    entry_points={
        "console_scripts": [
            # "look_to_point = src.lasr_skills.look_to_point:main",
            "get_image = lasr_skills.vision.get_image:main",
            "wait_state = lasr_skills.wait:main",
            "say = lasr_skills.say:main",
            "detect_3d = lasr_skills.detect_3d:main",
            "detect_all_in_polygon = lasr_skills.detect_all_in_polygon:main",
            "crop_image_3d = lasr_skills.vision.crop_image_3d:main",
        ],
    },
)
