#!/usr/bin/env python3

import rospy
from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest
from lasr_vision_msgs.msg import CDRequest
from geometry_msgs.msg import Polygon, Point32

def test_cropped_detection():
    rospy.init_node("test_cropped_detection")

    rospy.wait_for_service("/vision/cropped_detection")
    try:
        detect = rospy.ServiceProxy("/vision/cropped_detection", CroppedDetection)

        # OPTIONAL: A loose polygon to include the whole view
        poly = Polygon(points=[
            Point32(x=-10, y=-10), Point32(x=10, y=-10),
            Point32(x=10, y=10), Point32(x=-10, y=10)
        ])

        req = CroppedDetectionRequest(
            requests=[
                CDRequest(
                    method="closest",
                    use_mask=True,
                    yolo_model="yolov8x-seg.pt",
                    yolo_model_confidence=0.5,
                    yolo_nms_threshold=0.3,
                    return_sensor_reading=True,
                    object_names=["person"],
                    polygons=[poly],  # Or leave empty: []
                )
            ]
        )

        res = detect(req)

        print(f"Received {len(res.responses)} response(s).")
        if res.responses:
            r = res.responses[0]
            print(f"3D Detections: {len(r.detections_3d)}")
            print(f"Cropped images: {len(r.cropped_imgs)}")
            if r.detections_3d:
                print(f"First detection point: {r.detections_3d[0].point}")

    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    test_cropped_detection()
