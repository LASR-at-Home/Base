from setuptools import find_packages, setup
import os
from glob import glob

package_name = "lasr_vision_lang_sam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siyao",
    maintainer_email="sveali41@gmail.com",
    description="LASR Vision Lang Sam",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lang_sam_service_node = lasr_vision_lang_sam.nodes.lang_sam_service_node:main",
            "lang_sam_example = lasr_vision_lang_sam.examples.example:main"
        ],
    },
)