import rospy

import speech_recognition as sr

from queue import Queue
from abc import ABC, abstractmethod

# from .source import AudioTopic

class AbstractPhraseCollector(ABC):
    '''
    Supertype holding a queue of audio data representing a phrase
    '''

    data: Queue[bytes] = Queue()

    @abstractmethod
    def start(self):
        '''
        Start collecting phrases
        '''
        pass

    @abstractmethod
    def stop(self):
        '''
        Stop collecting phrases
        '''
        pass

    @abstractmethod
    def sample_rate(self):
        '''
        Sample rate of the data
        '''
        pass

    @abstractmethod
    def sample_width(self):
        '''
        Sample width of the data
        '''
        pass

class RecognizerPhraseCollector(AbstractPhraseCollector):
    '''
    Collect phrases using a SoundRecognition Recognizer

    This will monitor energy levels on the input and only
    capture when a certain threshold of activity is met.
    '''

    _recorder: sr.Recognizer
    _phrase_time_limit: float

    def _record_callback(self, _, audio: sr.AudioData) -> None:
        '''
        Collect raw audio data from the microphone
        '''
        self.data.put(audio.get_raw_data())

    def __init__(self, energy_threshold: int = 500, phrase_time_limit: float = 2) -> None:
        super().__init__()
        self._recorder = sr.Recognizer()
        self._recorder.dynamic_energy_threshold = False
        self._recorder.energy_threshold = energy_threshold
        self._phrase_time_limit = phrase_time_limit

    @abstractmethod
    def adjust_for_noise(self, source: sr.AudioSource):
        rospy.loginfo('Adjusting for background noise...')
        with source:
            self._recorder.adjust_for_ambient_noise(source)

    @abstractmethod
    def start(self, source: sr.AudioSource):        
        rospy.loginfo('Started source listen thread')
        self._stopper = self._recorder.listen_in_background(source, self._record_callback, phrase_time_limit=self._phrase_time_limit)

    def stop(self):
        self._stopper()

    def sample_rate(self):
        return self._source.SAMPLE_RATE

    def sample_width(self):
        return self._source.SAMPLE_WIDTH

class MicrophonePhraseCollector(RecognizerPhraseCollector):
    '''
    Collect phrases from the default microphone
    '''

    _source: sr.Microphone

    def __init__(self, energy_threshold: int = 500, phrase_time_limit: float = 2, device_index: int = None) -> None:
        self._source = sr.Microphone(device_index=device_index)
        super().__init__(energy_threshold, phrase_time_limit)

    def adjust_for_noise(self):
        return super().adjust_for_noise(self._source)

    def start(self):
        return super().start(self._source)

# class AudioTopicPhraseCollector(RecognizerPhraseCollector):
#     '''
#     Collect phrases from an audio topic
#     '''

#     _source: AudioTopic

#     def __init__(self, topic: str, energy_threshold: int = 100, phrase_time_limit: float = 2) -> None:
#         self._source = AudioTopic(topic)
#         super().__init__(energy_threshold, phrase_time_limit)

#     def start(self):
#         return super().start(self._source)
