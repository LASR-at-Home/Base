import os
import whisper  # type: ignore
from ament_index_python import packages
from rclpy.node import Node

# Keep all loaded models in memory
MODEL_CACHE = {}

class ModelCache(Node):
    def __init__(self):
        super().__init__('lasr_speech_recognition_whisper_cache')

    def load_model(
            self,
            name: str, device: str = "cpu", load_test_file: bool = False
    ) -> whisper.Whisper:
        """Loads a whisper model from disk, or from cache if it has already been loaded.

        Args:
            name (str): Name of the whisper model. Must be the name of an official whisper
            model, or the path to a model checkpoint.
            device (str, optional): Pytorch device to put the model on. Defaults to 'cpu'.
            load_test_file (bool, optional): Whether to run inference on a test audio file
            after loading the model (if model is not in cache). Defaults to False. Test file
            is assumed to be called "test.m4a" and be in the root of the package directory.

        Returns:
            whisper.Whisper: Whisper model instance
        """
        global MODEL_CACHE

        if name not in MODEL_CACHE:
            self.get_logger().info(f"Loading model {name}")
            MODEL_CACHE[name] = whisper.load_model(name, device=device)
            self.get_logger().info(f"Sucessfully loaded model {name} on {device}")
            if load_test_file:
                package_install = packages.get_package_prefix("lasr_speech_recognition_whisper")
                package_root = os.path.abspath(os.path.join(package_install, os.pardir, os.pardir, "lasr_speech_recognition_whisper"))
                example_fp = os.path.join(package_root, "test.m4a")
                self.get_logger().info(
                    "Running transcription on example file to ensure model is loaded..."
                )
                test_result: str = MODEL_CACHE[name].transcribe(
                    example_fp, fp16=device == "cuda"
                )
                self.get_logger().info(f"Transcription test result: {test_result}")

        return MODEL_CACHE[name]
