from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lasr_vision_deepface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siyao',
    maintainer_email='sveali41@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "deepface_service_node = lasr_vision_deepface.nodes.deepface_services:main",
            "deepface_node = lasr_vision_deepface.deepface:main",
            "recognise_face = lasr_vision_deepface.examples.recogniseFace:main",
        ],
    },
)
