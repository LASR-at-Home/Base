from glob import glob
from setuptools import find_packages, setup

package_name = "lasr_wakewords"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Siyao Li",
    maintainer_email="sveali41@gmail.com",
    description="Wakeword detection service built on openWakeWord",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wakeword_service = lasr_wakewords.wakeword_service:main",
        ]
    },
)
