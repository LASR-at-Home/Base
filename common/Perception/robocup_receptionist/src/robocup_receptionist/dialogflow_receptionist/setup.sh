#dialogflow setup
cd /catkin_ws/src
git clone https://github.com/piraka9011/dialogflow_ros.git
cd dialogflow_ros
pip install -r requirements.txt
cd ..
cd robocup/LASR
export GOOGLE_APPLICATION_CREDENTIALS='/home/elisabeth/robocup/src/LASR/common/ExternalPackages/dialogflow_speech/config/receptionist-345509.json'