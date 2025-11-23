from setuptools import find_packages, setup

package_name = 'tf_pcl'

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
    entry_points={
        'console_scripts': [
        ],
    },
)