from setuptools import setup

package_name = "tf_pcl"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        # Install package.xml in the appropriate location
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],  # Add any Python dependencies here
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="A ROS 2 package for point cloud transformations",
    license="Apache License 2.0",  # Update license as per your project
    tests_require=["pytest"],  # Add any test dependencies here
    entry_points={
        "console_scripts": [
            "pointcloud_transformer = tf_pcl.__init__:main",
        ],
    },
)
