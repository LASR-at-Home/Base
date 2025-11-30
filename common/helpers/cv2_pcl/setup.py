from setuptools import setup

package_name = "cv2_pcl"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jared Swift",
    maintainer_email="jared.swift@kcl.ac.uk",
    description="The cv2_pcl package for ROS2, providing utilities for working with OpenCV and PointClouds.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Add any scripts you want to expose as command-line executables here
            # For example: 'pcl_processor = cv2_pcl.pcl_processor:main'
        ]
    },
)
