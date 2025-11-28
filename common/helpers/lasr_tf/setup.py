from setuptools import find_packages, setup

package_name = "lasr_tf"

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
    maintainer="Aldrich Fernandes",
    maintainer_email="aldrich.fernandes@kcl.ac.uk",
    description="The lasr_tf package",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "service = lasr_tf.tf_server:main",
        ],
    },
)
