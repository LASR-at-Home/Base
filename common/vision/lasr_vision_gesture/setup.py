from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install

import os
from glob import glob

_here = os.path.dirname(os.path.abspath(__file__))

package_name = 'lasr_vision_gesture'

class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            scripts_base=self.install_scripts,
            install_base=self.install_base,
            package_name=package_name,
            python_version="3",
            source_dir=_here,
        )
        # instead of self.install_base we may also use:
        # self.config_vars['platbase'] or self.config_vars['base']
        # Exchange the python_version with '3' if your package uses Python3.
        return

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aldrich',
    maintainer_email='aldrich.fernandes05@gmail.com',
    description='A package providing a gesture detection service.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_service_node = lasr_vision_gesture.service:main',
        ],
    },
    cmdclass={"install": InstallCommand},
)
