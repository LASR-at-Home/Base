#!/usr/bin/env python3
import rospy
from listen_module.srv import DialogListen, DialogListenRequest, DialogListenResponse
from listen_module.subactions import ButtonAction, NavigationAction
import sounddevice as sd
import os, rospkg, re
from proto.marshal.collections.maps import MapComposite


# Audio recording parameters
INPUT_DEVICE = sd.query_hostapis()[0]['default_input_device']
DEVICE_INFO = sd.query_devices(INPUT_DEVICE, 'input')
RATE = int(DEVICE_INFO["default_samplerate"])  # Sampling rate in Hz
CHANNELS = 1  # Number of channels
BLOCK_SIZE = 4096  # Number of frames per block
FORMAT = "int16"  # Data type of audio samples
LANGUAGE_CODE = "en-GB"  # Language code of the query

RECORD_SECONDS = 10
GOOGLE_APPLICATION_CREDENTIALS = os.path.join(rospkg.RosPack().get_path('listen_module'), 'config',
                                              '{}.json'.format('newagent-groe'))


class DialogflowServer:
    def __init__(self):
        self.handler = {
            "BUTTON_PRESSED": {
                "id": "newagent-groe",
                "action": ButtonAction

            },
            "ROOM_REQUEST": {
                "id": "newagent-groe",
                "action": NavigationAction
            }
        }
        self.dialog_srv = rospy.Service("/dialogflow_server", DialogListen, self.handle_audio_cb)
        self.stream = sd.RawInputStream(samplerate=RATE, blocksize=BLOCK_SIZE, device=INPUT_DEVICE,
                                        dtype=FORMAT,
                                        channels=CHANNELS)



    def handle_audio_cb(self, request: DialogListenRequest):
        rospy.loginfo("Received request in dialog server")
        rospy.loginfo(request.query_text)
        resp = DialogListenResponse()


        action_id = self.handler[request.action]["id"]
        action = self.handler[request.action]["action"](action_id=action_id)
        result = action.subaction[request.subaction](audio_or_text=request.query_text)  # check if is text
        print(action, result, "action, subaction, result")
        try:
            if result:
                resp.status = True
                if isinstance(result, MapComposite) or isinstance(result, dict):
                    result = dict(result)
                    print(result, "result" + ("-" * 100))
                    result = str(result["name"])
                    print(result, "result" + ("-" * 100))
                if re.search(r'^[-+]?[0-9]*\.?[0-9]+$', str(result)) or result.isdigit():
                    result = float(result)
                    print(result)
                resp.result = str(result)
        except Exception as e:
            rospy.logerr(e)
            resp.status = False
            resp.result = str("Error in handling audio")
        return resp


if __name__ == '__main__':
    rospy.init_node('dialogflow_server')
    server = DialogflowServer()
    rospy.loginfo("Dialogflow server is ready to take input")
    rospy.spin()
