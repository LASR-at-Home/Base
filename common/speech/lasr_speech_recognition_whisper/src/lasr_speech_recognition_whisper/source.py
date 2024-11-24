import rclpy
from rclpy.node import Node
import pyaudio
import speech_recognition as sr

from audio_common_msgs.msg import AudioInfo, AudioData

from .bytesfifo import BytesFIFO

# TODO rospy.wait_for_message()


class AudioTopic(sr.AudioSource, Node):
    """
    Use a ROS topic as an AudioSource
    """

    _topic: str
    # _sub: node.create_subscription TODO add type if possible

    def __init__(self, topic: str, chunk_size=1024) -> None:
        Node.__init__(self, "source")

        self._topic = topic
        self.subscription = self.create_subscription(
            AudioInfo, f"{topic}/audio_info", self.callback, 10
        )
        # config: AudioInfo = rospy.wait_for_message(f"{topic}/audio_info", AudioInfo)
        self.config = None  # TODO test that this works
        if self.config is not None:
            assert self.config.coding_format == "wave", "Expected Wave audio format"
            assert self.config.sample_format == "S16LE", "Expected sample format S16LE"
            self.get_logger().info(self.config)

            self.SAMPLE_WIDTH = pyaudio.get_sample_size(pyaudio.paInt16)
            self.SAMPLE_RATE = self.config.sample_rate

            self.CHUNK = chunk_size
            self.stream = None

    def callback(self, msg):
        self.get_logger().info("Message received")
        self.config = msg

    def __enter__(self):
        """
        Start stream when entering with: block
        """

        assert (
            self.stream is None
        ), "This audio source is already inside a context manager"
        self.stream = BytesFIFO(1024 * 10)  # 10 kB buffer
        self._sub = self.node.create_subscription(
            AudioData, f"{self._topic}/audio", self._read
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Close out stream on exit
        """

        self.stream = None
        self.destroy_subscription(
            self._sub
        )  # TODO behaviour, was self._sub.unregister()

    def _read(self, msg: AudioData) -> None:
        """
        Forward raw audio data to queue
        """

        self.stream.write(msg.data)
