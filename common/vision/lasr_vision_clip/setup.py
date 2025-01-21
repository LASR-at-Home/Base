from setuptools import find_packages, setup

package_name = "lasr_vision_clip"

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
    description="Object recognition",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "clip_utils = scripts.clip_utils:main",
            "learn_face = scripts.learn_face:main",
            "vqa = lasr_vision_clip.nodes.vqa:main",
        ],
    },
)
