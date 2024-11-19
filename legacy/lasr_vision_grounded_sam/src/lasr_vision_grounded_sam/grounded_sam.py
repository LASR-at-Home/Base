import rospy
from autodistill_grounded_sam import GroundedSAM
from autodistill.detection import CaptionOntology

from cv2_img import msg_to_cv2_img
from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import GroundedSamRequest, GroundedSamResponse


def load_model() -> GroundedSAM:
    return GroundedSAM(
        ontology=CaptionOntology(
            {
                "7up can": "7up",
                "apple": "apple",
                "banana": "banana",
                "baseball": "baseball",
                "bowl": "bowl",
                "cheezit": "cheezit",
                "chocolate_jello": "chocolate_jello",
                "cleanser": "cleanser",
                "coffee_grounds": "coffee_grounds",
                "cola can": "cola",
                "cornflakes": "cornflakes",
                "cup": "cup",
                "dice": "dice",
                "fork": "fork",
                "iced tea bottle": "iced_tea",
                "juice pouch": "juice_pack",
                "knife": "knife",
                "lemon": "lemon",
                "milk carton": "milk",
                "yellow mustard bottle": "mustard",
                "orange": "orange",
                "orange juice carton": "orange_juice",
                "peach": "peach",
                "pear": "pear",
                "plate": "plate",
                "plum": "plum",
                "pringles": "pringles",
                "red wine bottle": "red_wine",
                "rubiks_cube": "rubiks_cube",
                "soccer_ball": "soccer_ball",
                "spam": "spam",
                "sponge": "sponge",
                "spoon": "spoon",
                "strawberry_jello": "strawberry_jello",
                "box of sugar": "sugar",
                "tennis_ball": "tennis_ball",
                "tomato_soup": "tomato_soup",
                "tropical juice bottle": "tropical_juice",
                "tuna": "tuna",
                "water bottle": "water",
            }
        )
    )


def run_grounded_sam(
    req: GroundedSamRequest, base_model: GroundedSAM
) -> GroundedSamResponse:

    base_model.ontology = CaptionOntology(
        {k: v for k, v in zip(req.queries, req.query_mappings)}
    )
    cv2_img = msg_to_cv2_img(req.image_raw)
    # run inference on a single image
    results = base_model.predict(cv2_img)
    classes = base_model.ontology.classes()
    detections: Detection = []

    rospy.loginfo("GROUNDED SAM DETECTION RESULTS:")
    for i in range(len(results)):
        detection = Detection()

        detection.name = classes[results.class_id[i]]
        detection.confidence = results.confidence[i]
        detection.xywh = [
            results.xyxy[i][0],
            results.xyxy[i][1],
            results.xyxy[i][2] - results.xyxy[i][0],
            results.xyxy[i][3] - results.xyxy[i][1],
        ]
        detection.xyseg = results.mask[i].flatten().tolist()
        detections.append(detection)

        rospy.loginfo(
            f"Detection {i}: {detection.name} with confidence {detection.confidence}"
        )

    return GroundedSamResponse(detections=detections)
