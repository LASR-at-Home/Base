#!/usr/bin/env python3
import sounddevice as sd
from proto.marshal.collections.maps import MapComposite
from google.cloud import dialogflow_v2 as dialogflow
import rospkg, os, rospy, time, uuid

# Define some parameters for the audio stream
INPUT_DEVICE = sd.query_hostapis()[0]['default_input_device']
print(INPUT_DEVICE)
DEVICE_INFO = sd.query_devices(INPUT_DEVICE, 'input')
RATE = int(DEVICE_INFO["default_samplerate"])   # Sampling rate in Hz, ERRORS might occur as Expressions of alsa
RATE = 22050
print(RATE)
CHANNELS = 1  # Number of channels
BLOCK_SIZE = 4096  # Number of frames per block
FORMAT = "int16"  # Data type of audio samples
LANGUAGE_CODE = "en-GB"  # Language code of the query
GOOGLE_APPLICATION_CREDENTIALS = os.path.join(rospkg.RosPack().get_path('listen_module'), 'config',
                                              '{}.json'.format('newagent-groe'))
# GOOGLE_APPLICATION_CREDENTIALS = "/home/nicole/robocup/rexy/src/rexy/listen_module/config/newagent-groe.json"

class StreamHandler:
    def __init__(self, session_client,session):
        self.session = session
        self.session_client = session_client
        self.stop_requested = False
        self.query_params = None


    def stop(self):
        self.stop_requested = True

    def audio_generator(self):
        start_time = time.time()
        rospy.loginfo("Starting stream handler")
        with sd.RawInputStream(samplerate=RATE, channels=CHANNELS, dtype=FORMAT) as self.stream:
            while self.stream.active:
                if self.stop_requested or rospy.is_shutdown() or time.time() - start_time > 10000:
                    rospy.loginfo("Stopping stream handler")
                    break
                data, overflowed = self.stream.read(BLOCK_SIZE)
                if overflowed:
                    print("Warning: some audio data was lost")
                yield bytes(data)

    # Define a generator function that yields Dialogflow requests
    def request_generator(self):
        # Create the initial request with the session and query input
        request = dialogflow.StreamingDetectIntentRequest(
            session=self.session,
            query_params=self.query_params,
            query_input=dialogflow.QueryInput(
                audio_config=dialogflow.InputAudioConfig(
                    audio_encoding=dialogflow.AudioEncoding.AUDIO_ENCODING_LINEAR_16,
                    sample_rate_hertz=RATE,
                    language_code=LANGUAGE_CODE,
                    single_utterance=True,
                ),
            ),
        )
        yield request

        # Loop over the audio blocks from the microphone
        for input_audio in self.audio_generator():
            # Create a request with the input audio
            request = dialogflow.StreamingDetectIntentRequest(input_audio=input_audio)
            yield request


    def get_responses(self, st, query_params=None):
        # send the requests to the dialogflow api and get the responses
        try:
            self.query_params = query_params
            self.stop_requested = False
            responses = st.session_client.streaming_detect_intent(st.request_generator())
            self.query_params = None
            return responses
        except Exception as e:
            rospy.loginfo(str(e) + " error in request of stream handler!")

if __name__ == '__main__':

    rospy.init_node('stream_handler')
    project_id = 'newagent-groe'
    session_id = str(uuid.uuid4())
    # project_credentials = os.path.join(rospkg.rospack().get_path('listen_module'), 'config',
    #                                         '{}.json'.format(project_id))
    project_credentials = "/home/nicole/robocup/rexy/src/rexy/listen_module/config/newagent-groe.json"
    session_client = dialogflow.SessionsClient.from_service_account_file(project_credentials)
    session = session_client.session_path(project_id, session_id)
    st = StreamHandler(session_client, session)
    # send the requests to the dialogflow api and get the responses
    try:
        responses = st.session_client.streaming_detect_intent(st.request_generator())
        print(responses)

        print("=" * 20)
        try:
            for response in responses:
                print(
                    'intermediate transcript: "{}".'.format(
                        response.recognition_result.transcript
                    )
                )

                # note: the result from the last response is the final transcript along
                # with the detected content.
                query_result = response.query_result

                print("=" * 20)
                print("query text: {}".format(query_result.query_text))
                print(
                    "detected intent: {} (confidence: {})\n".format(
                        query_result.intent.display_name, query_result.intent_detection_confidence
                    )
                )
                print("fulfillment text: {}\n".format(query_result.fulfillment_text))
                # get the parameters
                parameters = query_result.parameters
                if parameters:
                    for parameter_name, parameter_value in parameters.items():
                        print(f"parameter: {parameter_name} = {parameter_value}")
                        if isinstance(parameter_value, MapComposite):
                            parameter_values = dict(parameter_value)
                            print(parameter_values)
        except Exception as e:
            print(e, "error in response")
    except Exception as e:
        rospy.loginfo(str(e) + " error in request of stream handler!")

