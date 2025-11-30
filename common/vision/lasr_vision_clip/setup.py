from setuptools import find_packages, setup

package_name = "lasr_vision_clip"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"], where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaliyah",
    maintainer_email="aaliyah.merchant@kcl.ac.uk",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_vqa = lasr_vision_clip.nodes.test_vqa:main",
            "vqa = lasr_vision_clip.nodes.vqa:main",
        ]
    },
)
