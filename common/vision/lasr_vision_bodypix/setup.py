from setuptools import setup, find_packages


setup(
    name='lasr_vision_bodypix',
    version='0.0.0',
    packages=["lasr_vision_bodypix"], 
    package_dir={'': 'src'},  
    install_requires=['setuptools'],
    zip_safe=True,

    entry_points={
        'console_scripts': [
            'bodypix_service_node = lasr_vision_bodypix.nodes.bodypix_services:main',
            'bodypix_node = lasr_vision_bodypix.src.lasr_vision_bodypix.bodypix:main',
        ],
    }
)





