import torch
import rospy
import whisper
import speech_recognition as sr

from io import BytesIO
from time import sleep
from threading import Thread
from abc import ABC, abstractmethod
from tempfile import NamedTemporaryFile
from datetime import datetime, timedelta

from .collector import AbstractPhraseCollector

from lasr_speech_recognition_msgs.msg import Transcription

class SpeechRecognitionWorker(ABC):
    '''
    Collect and run inference on phrases to produce a transcription
    '''

    _collector: AbstractPhraseCollector
    _tmp_file: NamedTemporaryFile
    _model: whisper.Whisper
    _current_sample: bytes
    _phrase_start: datetime | None
    _maximum_phrase_length: timedelta
    _infer_partial: bool
    _stopped = True

    def __init__(self, collector: AbstractPhraseCollector, model: whisper.Whisper, maximum_phrase_length = timedelta(seconds=3), infer_partial = True) -> None:
        self._collector = collector
        self._tmp_file = NamedTemporaryFile().name
        self._model = model
        self._current_sample = bytes()
        self._phrase_start = None
        self._maximum_phrase_length = maximum_phrase_length
        self._infer_partial = infer_partial

    @abstractmethod
    def on_phrase(self, phrase: str, finished: bool) -> None:
        '''
        Handle a partial or complete transcription
        '''
        pass

    def _finish_phrase(self):
        '''
        Complete the current phrase and clear the sample
        '''

        text = self._perform_inference()
        if text is not None:
            self.on_phrase(text, True)

        self._current_sample = bytes()
        self._phrase_start = None

    def _perform_inference(self):
        '''
        Run inference on the current sample
        '''

        rospy.loginfo('Processing sample')
        audio_data = sr.AudioData(self._current_sample, self._collector.sample_rate(), self._collector.sample_width())
        wav_data = BytesIO(audio_data.get_wav_data())

        with open(self._tmp_file, 'w+b') as f:
            f.write(wav_data.read())

        rospy.loginfo('Running inference')        
        try:
            result = self._model.transcribe(self._tmp_file, fp16=torch.cuda.is_available())
        except RuntimeError:
            return None
        text = result['text'].strip()

        # Detect and drop garbage
        if len(text) == 0 or text.lower() in ['.', 'you', 'thanks for watching!']:
            self._phrase_start = None
            self._current_sample = bytes()
            rospy.loginfo('Skipping garbage...')
            return None
    
        return text

    def _worker(self):
        '''
        Indefinitely perform inference on the given data
        '''

        rospy.loginfo('Started inference worker')

        while not self._stopped:
            try:
                # Check whether the current phrase has timed out
                now = datetime.utcnow()
                if self._phrase_start and now - self._phrase_start > self._maximum_phrase_length:
                    rospy.loginfo('Reached timeout for phrase, ending now.')
                    self._finish_phrase()

                # Start / continue phrase if data is coming in
                if not self._collector.data.empty():
                    self._phrase_start = datetime.utcnow()

                    # Concatenate new data with current sample
                    while not self._collector.data.empty():
                        self._current_sample += self._collector.data.get()

                    rospy.loginfo('Received and added more data to current audio sample.')

                    # Run inference on partial sample if enabled
                    if self._infer_partial:
                        text = self._perform_inference()

                        # Handle partial transcription
                        if text is not None:
                            self.on_phrase(text, False)

                sleep(0.2)
            except KeyboardInterrupt:
                self._stopped = True

        rospy.loginfo('Worker finished')        

    def start(self):
        '''
        Start performing inference on incoming data
        '''
        
        assert self._stopped, "Already running inference"
        self._stopped = False
        self._collector.start()
        worker_thread = Thread(target=self._worker)
        worker_thread.start()
    
    def stop(self):
        '''
        Stop the worker from running inference
        '''

        assert not self._stopped, "Not currently running"
        self._collector.stop()
        self._stopped = True

        # clear next phrase
        self._current_sample = bytes()
        while not self._collector.data.empty():
            self._current_sample += self._collector.data.get()

class SpeechRecognitionToStdout(SpeechRecognitionWorker):
    '''
    Recognise speech and pass it through to standard output
    '''

    def on_phrase(self, phrase: str, finished: bool) -> None:
        rospy.loginfo('[' + ('x' if finished else ' ') + '] ' + phrase)

class SpeechRecognitionToTopic(SpeechRecognitionToStdout):
    '''
    Recognise speech and publish it to a topic
    '''

    _pub: rospy.Publisher

    def __init__(self, collector: AbstractPhraseCollector, model: whisper.Whisper, topic: str, maximum_phrase_length=timedelta(seconds=1), infer_partial=True) -> None:
        super().__init__(collector, model, maximum_phrase_length, infer_partial)
        rospy.loginfo(f'Will be publishing transcription to {topic}')
        self._pub = rospy.Publisher(topic, Transcription, queue_size=5)
    
    def on_phrase(self, phrase: str, finished: bool) -> None:
        super().on_phrase(phrase, finished)
        msg = Transcription()
        msg.phrase = phrase
        msg.finished = finished
        self._pub.publish(msg)
