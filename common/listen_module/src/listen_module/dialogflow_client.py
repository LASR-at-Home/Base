#!/usr/bin/env python3
import rospy, os, rospkg, uuid
from stream_handler import StreamHandler
import sounddevice as sd
from google.cloud import dialogflow_v2 as dialogflow

# Inspired by the LASR team dialogflow package! Thank you for teaching me so much!

INPUT_DEVICE = sd.query_hostapis()[0]['default_input_device']
DEVICE_INFO = sd.query_devices(INPUT_DEVICE, 'input')
RATE = int(DEVICE_INFO["default_samplerate"])  # Sampling rate in Hz
CHANNELS = 1  # Number of channels
BLOCK_SIZE = 4096  # Number of frames per block
FORMAT = "int16"  # Data type of audio samples
LANGUAGE_CODE = "en-GB"  # Language code of the query


class DialogClient:
    def __init__(self, project_id, session_id=str(uuid.uuid4())):
        self.project_id = project_id
        self.project_credentials = os.path.join(rospkg.RosPack().get_path('listen_module'), 'config',
                                                '{}.json'.format(self.project_id))
        self.session_id = session_id
        self.session_client = dialogflow.SessionsClient.from_service_account_file(self.project_credentials)
        self.session = self.session_client.session_path(self.project_id, self.session_id)
        self.session_stream = None

    def detect_intent(self, audio_or_text, query_params=None):
        if audio_or_text == "SOUND:PLAYING:PLEASE":
            self.session_stream = StreamHandler(self.session_client, self.session)
            return self.detect_audio(query_params)
        else:
            return self.detect_text(audio_or_text, query_params)

    def detect_text(self, text, query_params=None):
        text_input = dialogflow.types.TextInput(text=text, language_code=LANGUAGE_CODE)
        query_input = dialogflow.types.QueryInput(text=text_input)
        request = dialogflow.DetectIntentRequest(
            session=self.session,
            query_input=query_input,
            query_params=query_params
        )
        response = self.session_client.detect_intent(request=request)
        print(response, response.query_result.fulfillment_text, response.query_result.intent.display_name,
              "resp in text")
        return response


    def detect_audio(self, query_params=None):
        responses = self.session_stream.get_responses(self.session_stream, query_params)
        response = None
        for response in responses:
            print(response, "response in audio")
            if response.recognition_result.message_type == dialogflow.StreamingRecognitionResult.MessageType.END_OF_SINGLE_UTTERANCE:
                print("End of single utterance")
                self.stop()
            if not response.query_result.query_text:
                print("No query text")
        return response

    def stop(self):
        self.session_stream.stop()


if __name__ == '__main__':
    rospy.init_node('dialogflow_client')
    try:
        client = DialogClient("newagent-groe")
        # client.detect_intent("CAn you press button 7")
        client.detect_intent("SOUND:PLAYING:PLEASE")


    except Exception as e:
        print(e)
        print("Shutting down")
