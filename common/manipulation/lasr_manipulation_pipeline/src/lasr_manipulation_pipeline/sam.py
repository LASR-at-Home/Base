import ultralytics
import cv2
import numpy as np

import cv2_pcl

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2

from . import conversions


def segment(pcl: PointCloud2) -> PointCloud2:
    im = cv2_pcl.pcl_to_cv2(pcl)
    bbox = []
    drawing = False
    start_point = (0, 0)
    assert im is not None
    image_copy = im.copy()
    sam = ultralytics.FastSAM("FastSAM-s.pt").to("cpu")

    def mouse_callback_bbox(event, x, y, flags, param):
        nonlocal drawing, start_point, bbox, image_copy
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            start_point = (x, y)
            bbox = []
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            image_copy = im.copy()
            cv2.rectangle(image_copy, start_point, (x, y), (0, 255, 0), 2)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            end_point = (x, y)
            bbox = [start_point, end_point]

    while True:
        bbox = []
        cv2.namedWindow("Image")
        cv2.setMouseCallback("Image", mouse_callback_bbox)
        while True:
            cv2.imshow("Image", image_copy)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if bbox:
                break
        cv2.destroyAllWindows()

        if not bbox:
            continue

        x0, y0 = bbox[0]
        x1, y1 = bbox[1]
        box = [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]
        result = sam.predict(
            im,
            bboxes=[box],
            retina_masks=True,
            iou=0.25,
            conf=0.5,
        )[0]

        cv2.imshow("result", result.plot())
        print("Press Y to proceed, or N to re-segment")
        key = None
        while key not in [ord("y"), ord("n")]:
            key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()

        proceed = key == ord("y")
        if proceed:
            break

    xyseg = np.array(result.masks.xy).flatten().round().astype(int).reshape(-1, 2)
    contours = xyseg.reshape(-1, 2)
    mask = np.zeros(shape=im.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, pts=[contours], color=255)
    indices = np.argwhere(mask)

    pcl_xyz = np.array(
        list(
            point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=False)
        ),
        dtype=np.float32,
    ).reshape(im.shape[0], im.shape[1], 3)

    masked_points = pcl_xyz[indices[:, 0], indices[:, 1]]
    masked_points = masked_points[~np.isnan(masked_points).any(axis=1)]
    masked_points = masked_points[~np.all(masked_points == 0, axis=1)]
    masked_cloud = conversions.np_to_ros(masked_points, pcl.header.frame_id)

    return masked_cloud
