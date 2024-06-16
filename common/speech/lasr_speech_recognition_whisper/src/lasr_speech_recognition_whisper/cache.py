import os

import whisper  # type: ignore
import rospkg  # type: ignore
import rospy

# Keep all loaded models in memory
MODEL_CACHE = {}


def load_model(
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
        rospy.loginfo(f"Loading model {name}")
        MODEL_CACHE[name] = whisper.load_model(name, device=device)
        rospy.loginfo(f"Sucessfully loaded model {name} on {device}")
        if load_test_file:
            package_root = rospkg.RosPack().get_path("lasr_speech_recognition_whisper")
            example_fp = os.path.join(package_root, "test.m4a")
            rospy.loginfo(
                "Running transcription on example file to ensure model is loaded..."
            )
            test_result: str = MODEL_CACHE[name].transcribe(
                example_fp, fp16=device == "cuda"
            )
            rospy.loginfo(f"Transcription test result: {test_result}")

    return MODEL_CACHE[name]
