import rclpy
import pyaudio
import speech_recognition as sr

from audio_common_msgs.msg import AudioInfo, AudioData

from .bytesfifo import BytesFIFO

# TODO rospy.wait_for_message()

class AudioTopic(sr.AudioSource):
    """
    Use a ROS topic as an AudioSource
    """

    _topic: str
    # _sub: node.create_subscription TODO add type if possible

    def __init__(self, topic: str, chunk_size=1024) -> None:
        with rclpy.init(args=None):
            self.node = rclpy.create_node('source')

        self._topic = topic

        config: AudioInfo = rospy.wait_for_message(f"{topic}/audio_info", AudioInfo)
        assert config.coding_format == "wave", "Expected Wave audio format"
        assert config.sample_format == "S16LE", "Expected sample format S16LE"
        self.node.get_logger().info(config)

        self.SAMPLE_WIDTH = pyaudio.get_sample_size(pyaudio.paInt16)
        self.SAMPLE_RATE = config.sample_rate

        self.CHUNK = chunk_size
        self.stream = None

    def __enter__(self):
        """
        Start stream when entering with: block
        """

        assert (
            self.stream is None
        ), "This audio source is already inside a context manager"
        self.stream = BytesFIFO(1024 * 10)  # 10 kB buffer
        self._sub = self.node.create_subscription(AudioData, f"{self._topic}/audio",  self._read)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Close out stream on exit
        """

        self.stream = None
        self.node.destroy_subscription(self._sub)  # TODO behaviour, was self._sub.unregister()

    def _read(self, msg: AudioData) -> None:
        """
        Forward raw audio data to queue
        """

        self.stream.write(msg.data)
