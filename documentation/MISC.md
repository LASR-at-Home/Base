#### Misc Documentation for Porting to ROS2

##### rospy.wait_for_message()
Need to create a subscription and check if message is received

##### rospy.get_param()
Use rclpy's `node.get_parameter()` instead. Returns a Parameter object, so value needs to be accessed with:
`node.get_parameter('param_name').get_parameter_value()` and the correct type, e.g. `.string_value`
See [Play Motion skill for an example](../skills/src/lasr_skills/play_motion.py)