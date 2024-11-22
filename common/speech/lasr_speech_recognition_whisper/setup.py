from setuptools import find_packages, setup

package_name = 'lasr_speech_recognition_whisper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name, f"{package_name}.lasr_speech_recognition_whisper", f"{package_name}.src"],
    # package_dir={
    #     '': '.',
    #     package_name: os.path.join(package_name),
    #     f"{package_name}.whisper": os.path.join(package_name, 'whisper'),
    #     f"{package_name}.src": os.path.join(package_name, 'src'),
    # },
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
            'transcribe_microphone_server = lasr_speech_recognition_whisper.transcribe_microphone_server:main',
            'transcribe_microphone = lasr_speech_recognition_whisper.transcribe_microphone:main',
            'simple_transcribe_microphone = lasr_speech_recognition_whisper.simple_transcribe_microphone:main',
            'list_microphones = scripts.list_microphones:main',
            'microphone_tuning_test = scripts.microphone_tuning_test:main',
            'test_microphones = scripts.test_microphones:main',
            'test_speech_server = scripts.test_speech_server:main',
        ],
    },
)
