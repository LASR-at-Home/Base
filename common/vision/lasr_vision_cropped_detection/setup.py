from setuptools import setup, find_packages

setup(
    name="lasr_vision_cropped_detection",
    version="0.0.0",
    packages=find_packages(
        "src"
    ),  # Specify 'src' to find packages in the src directory
    package_dir={"": "src"},  # Maps the root package to the 'src' directory
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Benteng Ma",
    maintainer_email="benteng.ma@kcl.ac.uk",
    description="Description of lasr_vision_cropped_detection package",
    entry_points={
        "console_scripts": [
            "cropped_detection_service_node = lasr_vision_cropped_detection.nodes.cropped_detection_service:main",
            "cropped_detection_node = lasr_vision_cropped_detection.cropped_detection:main",
        ]
    },
)
