#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lasr_vision_clip.scripts.clip_utils import load_model, query_image
from lasr_vision_msgs.srv import VqaRequest, VqaResponse, Vqa
from sensor_msgs.msg import Image


class VqaService(Node):
    def __init__(self, model_device: str = "cuda") -> None:
        """Caches the clip model.

        Args:
            model_device (str, optional): device to load model onto. Defaults to "cuda".

        """
        super().__init__("clip_vqa_service")

        self._model = load_model(model_device)
        self._debug_pub = self.create_publisher(Image, "/clip_vqa/debug", 1)
        self.get_logger().info("Clip VQA service started")
        self._srv = self.create_service(Vqa, "/clip_vqa/query_service", self.query_clip)
        print("intialising")

    def query_clip(self, request: VqaRequest, response: VqaResponse) -> VqaResponse:
        """Queries CLIP from the robot's image stream and returns
        the most likely answer and cosine similarity score.

        Args:
            possible_answers (List[str]): set of possible answers.

        Returns:
            VqaResult
        """
        possible_answers = request.possible_answers
        answer, cos_score, annotated_img = query_image(
            request.image_raw, self._model, possible_answers, annotate=True
        )

        self._debug_pub.publish(annotated_img)

        response.answer = answer
        self.get_logger().info(f"Answer: {answer}")
        response.similarity = float(cos_score)
        return response


def main(args=None):
    rclpy.init(args=args)
    service = VqaService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info("Shutting down Clip VQA service...")
    finally:
        service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
