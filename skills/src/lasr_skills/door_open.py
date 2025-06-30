def select_depth_with_ratio(points, percentage = 75):
    depths = numpy.array(points)
    depths = depths[depths > 0]
    if len(depths) == 0:
        return None
    return numpy.percentile(depths, percentage)

class AnalyseDepthState(smach.State):
    def __init__(self, camera_topic: str = "/xtion/depth_registered/camera_info"):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], input_keys=["sam_detections"], output_keys=["sam_detections"])
        self.camera_topic = camera_topic

    def execute(self, userdata):
        raw_points = userdata.sam_detections.get("raw_points", [])
        filtered_points = userdata.sam_detections.get("filtered_points", [])

        if not raw_points and not filtered_points:
            rospy.logwarn("[AnalyseDepthState] No points to process.")
            return "failed"

        if not filtered_points:
            rospy.logwarn("[AnalyseDepthState] No filtered points to process.")
        else:
            try:
                rospy.loginfo(f"[AnalyseDepthState] Waiting for {self.camera_topic}")
                camera_info_msg = rospy.wait_for_message(self.camera_topic, CameraInfo, timeout=5)
                rospy.loginfo(f"[AnalyseDepthState] Received {self.camera_topic}")

                fx = camera_info_msg.K[0]
                fy = camera_info_msg.K[4]
                cx = camera_info_msg.K[2]
                cy = camera_info_msg.K[5]
            except rospy.ROSException:
                rospy.logerr(f"[AnalyseDepthState] Timeout: {self.camera_topic}")
                return "failed"
            
            handle_point_xy = numpy.min(userdata.sam_detections["mask"])
            handle_point_z = numpy.madien(filtered_points[])

        if not raw_points:
            rospy.logwarn("[AnalyseDepthState] No raw points to process.")

        if raw_points:
            z_values = [p[2] for p 



in raw_points if not.np.isnan(p[2])]
            if not z_values:
                rospy.logwarn("[EstimateAverageDepthState] All depth values are NaN.")
                userdata.sam_detections["average_depth"] = None
                return "done"
            
            door_depth = select_depth_with_ratio(z_values, 75)
            
