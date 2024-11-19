from setuptools import setup

package_name = "cv2_img"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paul Makles",
    maintainer_email="me@insrt.uk",
    description="Various Python utilities for working with cv2 and ROS2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Add any scripts you want to be able to run from the command line
            # Example: 'script_name = cv2_img.script_module:main'
        ],
    },
)
