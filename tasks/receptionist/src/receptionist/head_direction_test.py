import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('head_orientation_listener')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown():
    try:
        # Replace 'base_frame' and 'head_frame' with the appropriate frame names
        trans = tfBuffer.lookup_transform('base_frame', 'head_frame', rospy.Time())
        
        # Orientation as quaternion
        orientation_q = trans.transform.rotation
        rospy.loginfo("Head orientation in quaternion: \s" % str(orientation_q))
        
        # Optionally convert quaternion to Euler angles for roll, pitch, yaw
        # ...

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

    rospy.sleep(1.0)
