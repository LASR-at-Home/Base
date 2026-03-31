from setuptools import find_packages, setup

package_name = 'tf_pcl'
import setuptools.command.install
import ament_virtualenv.install

class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
            package_name=package_name,
            python_version='3'
        )
        # instead of self.install_base we may also use:
        # self.config_vars['platbase'] or self.config_vars['base']
        # Exchange the python_version with '3' if your package uses Python3.
        return

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={"": "src"},
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    maintainer='siyao',
    maintainer_email='sveali41@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    cmdclass={'install': InstallCommand
    },
    entry_points={
        'console_scripts': [
        ],
    },
)