#!/usr/bin/env python3

from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))

package_name = "skills"


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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
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
            "get_image = lasr_skills.vision.get_image:main",
            "wait_state = lasr_skills.wait:main",
            "say = lasr_skills.say:main",
            "detect_3d = lasr_skills.detect_3d:main",
            "detect_all_in_polygon = lasr_skills.detect_all_in_polygon:main",
            "crop_image_3d = lasr_skills.vision.crop_image_3d:main",
            "play_motion = lasr_skills.play_motion:main",
            "look = lasr_skills.look_to_point:main",
            "go_to_location = lasr_skills.go_to_location:main",
            "set_initial_pose = lasr_skills.set_initial_pose:main",
            "ask_and_listen = lasr_skills.ask_and_listen:main",
            "receive_object = lasr_skills.receive_object:main",
            "handover_object = lasr_skills.handover_object:main",
            "follow_person = lasr_skills.follow_person:main",
            "rotate = lasr_skills.rotate:main",
            "detect_keypoints_3d = lasr_skills.detect_keypoints_3d:main",
            "detect_wave = lasr_skills.detect_wave:main",
            "safety = lasr_skills.safety:main"
        ],
    },
)
