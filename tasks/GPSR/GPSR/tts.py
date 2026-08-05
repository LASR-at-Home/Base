import subprocess
import tempfile
import rclpy.node

from lasr_skills import Say

from yasmin import Blackboard


def say(node: rclpy.node.Node, text: str, bb: Blackboard):
    """Speak text using gtts in simulation, or robot TTS action otherwise."""
    if not text:
        return
    node.get_logger().info(text)
    simulation = node.get_parameter("simulation").get_parameter_value().bool_value

    if simulation:
        _say_gtts(text)
    else:
        _say_robot(node, text, bb)


def _say_gtts(text: str):
    try:
        from gtts import gTTS
        from pydub import AudioSegment

        tts = gTTS(text=text, lang="en")
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            mp3_path = f.name
        tts.save(mp3_path)
        wav_path = mp3_path.replace(".mp3", ".wav")
        AudioSegment.from_mp3(mp3_path).export(wav_path, format="wav")
        subprocess.run(["aplay", wav_path], check=False)
    except Exception as e:
        print(f"[TTS] gtts error: {e}", flush=True)


def _say_robot(node: rclpy.node.Node, text: str, bb: Blackboard):
    try:
        outcome = Say(text=text)(bb)

        # from tts_msgs.action import TTS
        # from rclpy.action import ActionClient
        # import threading

        # client = ActionClient(node, TTS, "/tts_engine/tts")
        # if not client.wait_for_server(timeout_sec=3.0):
        #     node.get_logger().error("TTS action server not available")
        #     return
        # goal = TTS.Goal()
        # goal.input = text
        # goal.locale = "en_GB"
        # done = threading.Event()
        # client.send_goal_async(goal).add_done_callback(
        #     lambda f: f.result()
        #     .get_result_async()
        #     .add_done_callback(lambda _: done.set())
        # )
        # done.wait()
    except Exception as e:
        node.get_logger().error(f"Robot TTS error: {e}")
