from setuptools import find_packages, setup

package_name = 'lasr_tf'

setup(
    name='lasr_tf',
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fadi',
    maintainer_email='fadimostefai@gmail.com',
    description='The lasr_tf package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_server = lasr_tf.tf_server:main'
        ],
    },
)
