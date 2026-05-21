import os
from setuptools import find_packages, setup
import setuptools.command.install
import ament_virtualenv.install


_here = os.path.dirname(os.path.abspath(__file__))

package_name = "lasr_speech_recognition_whisper"


class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(
            install_base=self.install_base,
            scripts_base=self.install_scripts,
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
    cmdclass={"install": InstallCommand},
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maayan",
    maintainer_email="maayan.armony@gmail.com",
    description="Speech recognition implemented using OpenAI Whisper",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "transcribe_microphone_server = lasr_speech_recognition_whisper.transcribe_microphone_server:main",
            "transcribe_microphone = lasr_speech_recognition_whisper.transcribe_microphone:main",
            "simple_transcribe_microphone = lasr_speech_recognition_whisper.simple_transcribe_microphone:main",
            "list_microphones = scripts.list_microphones:main",
            "microphone_tuning_test = scripts.microphone_tuning_test:main",
            "test_microphones = scripts.test_microphones:main",
            "test_speech_server = scripts.test_speech_server:main",
        ]
    },
)
