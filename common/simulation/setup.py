import os
from glob import glob
from setuptools import find_packages, setup

package_name = "simulation"


def share(*parts: str) -> str:
    return os.path.join("share", package_name, *parts)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [
                os.path.join("resource", package_name),
                os.path.join("resource", "pal_gazebo_worlds_private"),
            ],
        ),
        (share(), ["package.xml"]),
        (share("launch"), glob(os.path.join("launch", "*.launch.py"))),
        (share("worlds"), glob(os.path.join("worlds", "*"))),
        (share("maps"), glob(os.path.join("maps", "*"))),
        (share("config"), glob(os.path.join("config", "*"))),
        # Register as pal_gazebo_worlds_private so tiago_gazebo finds our worlds
        (
            os.path.join("share", "pal_gazebo_worlds_private", "worlds"),
            glob(os.path.join("worlds", "*")),
        ),
        *[
            (
                share(dirpath),
                [os.path.join(dirpath, f) for f in filenames if not f.startswith(".")],
            )
            for dirpath, _, filenames in os.walk("models")
            if any(not f.startswith(".") for f in filenames)
        ],
    ],
    install_requires=["setuptools"],
    maintainer="Michele Brienza",
    maintainer_email="brienza@diag.uniroma1.it",
    description="Tiago world",
    license="Apache-2.0",
    tests_require=[],
    entry_points={},
    zip_safe=True,
)
