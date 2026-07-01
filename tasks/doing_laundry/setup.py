from glob import glob

from setuptools import find_packages, setup

package_name = "doing_laundry"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("doing_laundry/models/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lasr",
    maintainer_email="lasr@gmail.com",
    description="Laundry pick-and-place: basket perception + grasp/place states.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "state_machine = doing_laundry.state_machine:main",
            "test_detect = doing_laundry.test_detect:main",
        ],
    },
)
