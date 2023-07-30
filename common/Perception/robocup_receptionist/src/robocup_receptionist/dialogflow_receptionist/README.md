
# Project Title

A brief description of what this project does and who it's for

The receptionist API contains method that can be used to facilitate conversation
in the robot

it mainly sends out DialogGoal to the server, which will be processed by the 
DialogFlowServer 

Dialogflow setup
    export GOOGLE_APPLICATION_CREDENTIALS="../config/appointmentscheduler-ldmb.json" 


To test, simply go to the python file and comment out 1 (emphasis 1) of the method
if __name__ == "__main__":
    
    api = ReceptionistAPI()

    #api.start_conversation()    
    api.get_name()
    #api.get_favourite_drink()
    #api.speak("Hi how are you?")
    rospy.spin()

Only test 1 method per test run! Testing methods all together in a single run has
not been tested before! 
(I have coded the code in a way that assumes that each method are called with a small time gap)

To run everything:
1)roslaunch dialogflow_speech dialog.launch
2)roslaunch microphone microphone.launch
3)rosrun robocup_receptionist receptionist_conversation_API.python

To enable output speech in a LAPTOP: 
<not really necessary since we are runnning on Tiago>

INSTALLATION FOR HMI_ROBIN
<Do this if you do not have the HMI_ROBIN package>
git clone https://github.com/gerardcanal/hmi_robin.git --branch=kinetic-devel

SETUP
in python file:
common/ExternalPackages/dialogflow_speech/src/dialogflow_speech/dialogflow_server.py

at line 19 under __init__ function, change
        use_tts = rospy.get_param('~use_tts', 2)  
    to
        use_tts = rospy.get_param('~use_tts', 0) 

RUNNING
1)roslaunch speech_database speech.launch 
2)rosrun dialogflow_speech speech_controller.py
