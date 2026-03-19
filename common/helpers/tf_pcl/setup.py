from setuptools import setup
import setuptools.command.install
import ament_virtualenv.install

package_name = "tf_pcl"

class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version='3'
        )
        
        return

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    install_requires=["setuptools"],  # Add any Python dependencies here
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="A ROS 2 package for point cloud transformations",
    license="Apache License 2.0",  # Update license as per your project
    tests_require=["pytest"],  # Add any test dependencies here
    cmdclass={'install': InstallCommand},
    entry_points={"console_scripts": ["pointcloud_transformer = tf_pcl.__init__:main"]},
)
