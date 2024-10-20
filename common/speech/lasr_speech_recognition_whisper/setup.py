from setuptools import find_packages, setup

package_name = 'lasr_speech_recognition_whisper'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=["lasr_speech_recognition_whisper"],
    package_dir = {'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maayan',
    maintainer_email='maayan.armony@gmail.com',
    description='Speech recognition implemented using OpenAI Whisper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transcribe_mic_server_node = lasr_speech_recognition_whisper.nodes.transcribe_microphone_server:main',
        ],
    },
)
