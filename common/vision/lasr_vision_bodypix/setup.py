from setuptools import setup, find_packages

setup(
    name='lasr_vision_bodypix',
    version='0.0.0',
    packages=find_packages('src'),  # Specify 'src' to find packages in the src directory
    package_dir={'': 'src'},  # Maps the root package to the 'src' directory
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Siyao Li',
    maintainer_email='sveali41@gmail.com',
    description='Description of lasr_vision_bodypix package',

    entry_points={
        'console_scripts': [
            'bodypix_service_node = lasr_vision_bodypix.nodes.bodypix_services:main',
            'bodypix_node = lasr_vision_bodypix.bodypix:main',
            'keypoint_relay = lasr_vision_bodypix.examples.keypoint_relay:main',  
            'mask_relay = lasr_vision_bodypix.examples.mask_relay:main'
        ],
    }
)



