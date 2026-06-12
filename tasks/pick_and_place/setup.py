from setuptools import find_packages, setup

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        ('share/' + package_name + '/launch', ['launch/pick_and_place.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yara',
    maintainer_email='yaralkhelaiwi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "state_machine = pick_and_place.state_machine:main",
            "test_detect = pick_and_place.test_detect:main",
            "point_head_stub = pick_and_place.point_head_stub:main",
        ],
    },
)
