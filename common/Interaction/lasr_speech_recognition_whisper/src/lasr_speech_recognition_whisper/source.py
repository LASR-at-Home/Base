import rospy
import pyaudio
import speech_recognition as sr

from audio_common_msgs.msg import AudioInfo, AudioData

from .bytesfifo import BytesFIFO

class AudioTopic(sr.AudioSource):
    '''
    Use a ROS topic as an AudioSource
    '''

    _topic: str
    _sub: rospy.Subscriber

    def __init__(self, topic: str, chunk_size = 1024) -> None:
        self._topic = topic

        config: AudioInfo = rospy.wait_for_message(f'{topic}/audio_info', AudioInfo)
        assert config.coding_format == 'wave', "Expected Wave audio format"
        assert config.sample_format == 'S16LE', "Expected sample format S16LE"
        rospy.loginfo(config)

        self.SAMPLE_WIDTH = pyaudio.get_sample_size(pyaudio.paInt16)
        self.SAMPLE_RATE = config.sample_rate

        self.CHUNK = chunk_size
        self.stream = None

    def __enter__(self):
        '''
        Start stream when entering with: block
        '''

        assert self.stream is None, "This audio source is already inside a context manager"
        self.stream = BytesFIFO(1024 * 10) # 10 kB buffer
        self._sub = rospy.Subscriber(f'{self._topic}/audio', AudioData, self._read)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Close out stream on exit
        '''

        self.stream = None
        self._sub.unregister()
    
    def _read(self, msg: AudioData) -> None:
        '''
        Forward raw audio data to queue
        '''

        self.stream.write(msg.data)
