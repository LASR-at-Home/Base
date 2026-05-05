from setuptools import find_packages, setup

package_name = "autonomous_behaviour"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/autonomous_behaviour.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michele",
    maintainer_email="michelebrienza1997@gmail.com",
    description="Autonomous behaviour task",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "state_machine=autonomous_behaviour.state_machine:main",
        ],
    },
)
