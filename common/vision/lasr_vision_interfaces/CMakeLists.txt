cmake_minimum_required(VERSION 3.14.4)
project(lasr_vision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Needed for built-in ROS types like Time

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Detection.msg"
  "msg/Detection3D.msg"
  "msg/BodyPixKeypoint.msg"
  "msg/BodyPixKeypointNormalized.msg"
  "msg/BodyPixMask.msg"
  "msg/CDRequest.msg"
  "msg/CDResponse.msg"
  "msg/DetectionArray.msg"
  "msg/Detection3DArray.msg"
  "msg/Sam2BboxWithFlag.msg"
  "msg/Sam2PointsWithLabelsAndFlag.msg"
  "msg/Sam2PromptArrays.msg"
  "msg/Keypoint.msg"
  "msg/Keypoint3D.msg"
  "msg/KeypointList.msg"
  "msg/Keypoint3DList.msg"
  "msg/LangSamDetection.msg"
  
  # Services
  "srv/AddFace.srv"
  "srv/YoloDetection.srv"
  "srv/YoloDetection3D.srv"
  "srv/YoloPoseDetection.srv"
  "srv/YoloPoseDetection3D.srv"
  "srv/BodyPixMaskDetection.srv"
  "srv/BodyPixKeypointDetection.srv"
  "srv/Recognise.srv"
  "srv/Recognise3D.srv"
  "srv/LearnFace.srv"
  "srv/Vqa.srv"
  "srv/PointingDirection.srv"
  "srv/TorchFaceFeatureDetectionDescription.srv"
  "srv/DetectFaces.srv"
  "srv/CheckKnownPeople.srv"
  "srv/DetectWave.srv"
  "srv/CroppedDetection.srv"
  "srv/ClipImageEncoder.srv"
  "srv/ClipLearnFace.srv"
  "srv/ClipRecogniseFace.srv"
  "srv/GroundedSam.srv"
  "srv/LangSam.srv"
  "srv/OpenVocabDetect.srv"
  "srv/OpenVocabDetectAndSegment.srv"
  
  # Actions
  "action/EyeTracker.action"
  
  DEPENDENCIES 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
