from setuptools import find_packages, setup

package_name = "lasr_llm"

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
    maintainer="Yara",
    maintainer_email="yara.alkhelaiwi@kcl.ac.uk",
    description="The lasr_llm package",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # "llm = lasr_llm.nodes.llm:main",
            "receptionist_service = lasr_llm.nodes.receptionist_service:main",
            "hri_task_service = lasr_llm.nodes.hri_task_service:main",
            "storing_groceries_service = lasr_llm.nodes.storing_groceries_service:main",
        ],
    },
)
