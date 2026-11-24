import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lasr_person_following"


def share(*parts):
    return os.path.join("share", package_name, *parts)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share(), ["package.xml"]),
        (share("launch"), glob("launch/*.py")),
        (share("config"), [f for f in glob("config/*") if os.path.isfile(f)]),
        (share("config", "scenarios"), glob("config/scenarios/*.yaml")),
        (share("behavior_trees"), glob("behavior_trees/*.xml")),
        (share("scripts"), glob("scripts/*.sh") + glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Person tracking and Nav2-based person following",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_tracker = lasr_person_following.person_tracker_node:main",
            "follow_manager = lasr_person_following.follow_manager_node:main",
            "scripted_person = lasr_person_following.sim.scripted_person:main",
            "follow_evaluator = lasr_person_following.sim.follow_evaluator:main",
            "follow_driver = lasr_person_following.sim.follow_driver:main",
            "sim_stubs = lasr_person_following.sim.sim_stubs:main",
            "reset_episode = lasr_person_following.sim.reset_episode:main",
        ],
    },
)
