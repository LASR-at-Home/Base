#!/usr/bin/env python3

from setuptools import find_packages, setup

package_name = "skills"
# setup_args = generate_distutils_setup(packages=["lasr_skills"], package_dir={"": "src"})

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # packages=["lasr_skills"],
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
    entry_points={
        "console_scripts": [
            # "look_to_point = src.lasr_skills.look_to_point:main",
        ],
    },
)
