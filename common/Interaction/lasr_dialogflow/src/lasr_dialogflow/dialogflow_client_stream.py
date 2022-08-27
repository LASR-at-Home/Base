#!/usr/bin/env python3

import os
import sounddevice
import dialogflow
import rospkg

CHUNK_SUZE = 4096

class DialogflowClientStream():

    def __init__(self, project_id, session_id, language_code="en", audio_encoding=dialogflow.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16, sample_rate=None, input_device=None):

        self.input_device = input_device
        self.sample_rate = sample_rate

        credentials_path = os.path.join(rospkg.RosPack().get_path('lasr_dialogflow'), 'config', '{}.json'.format(project_id))

        self.session_client = dialogflow.SessionsClient.from_service_account_file(credentials_path)
        self.session_path = self.session_client.session_path(project_id, session_id)
        self.audio_config = dialogflow.types.session_pb2.InputAudioConfig(
            language_code=language_code,
            audio_encoding=audio_encoding,
            sample_rate_hertz=sample_rate,
        )

    def response_generator(self, query_params=None):
        return self.session_client.streaming_detect_intent(self.request_generator(query_params))
    
    def requests_generator(self, query_params=None):
        self.stop_requested = False

        microphone_stream = sounddevice.InputStream(samplerate=self.sample_rate, device=self.input_device)

        query_input = dialogflow.types.session_pb2.QueryInput(audio_config=self.audio_config)

        # First request is for config
        yield dialogflow.types.session_pb2.StreamingDetectIntentRequest(
            session=self.session_path,
            query_input=query_input,
            query_params=query_params,
            single_utterance=True,
        )

        while not self.stop_requested and microphone_stream.active():
            data = microphone_stream.read(CHUNK_SIZE)
            yield dialogflow.types.session_pb2.StreamingDetectIntentRequest(input_audio=data)
        
        microphone_stream.stop()

    def text_request(self, text, query_params=None):
        text_input = dialogflow.types.session_pb2.TextInput(text=text, language_code="en-GB")
        query_input = dialogflow.types.session_pb2.QueryInput(text=text_input)
        response = self.session_client.detect_intent(session=self.session_path, query_input=query_input, query_params=query_params)

        return response
        
    def stop(self):
        self.stop_requested = True
