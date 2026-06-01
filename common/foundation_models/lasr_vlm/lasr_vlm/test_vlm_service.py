#!/usr/bin/env python3
"""
Two-layer test for the VLM describe-people pipeline:
  1. Unit-test parse_vlm_response directly (no ROS, no GPU)
  2. End-to-end ROS service client (requires service + Ollama running)

Usage:
  # Unit tests only:
  python3 test_vlm_describe_people.py --unit

  # End-to-end service call:
  python3 test_vlm_describe_people.py --e2e /path/to/person.jpg
"""

import sys
import argparse


# ─── Layer 1: unit-test the parser ────────────────────────────────────────────


def test_parser():
    """Test parse_vlm_response against realistic model outputs without needing ROS."""
    # Import directly from the inference module (no ROS needed)
    from lasr_vlm.vlm_inference import parse_vlm_response

    attributes = ["hair_color", "hair_length", "glasses", "hat", "shirt color"]

    cases = [
        # (description, raw_response, expected_dict)
        (
            "clean key:value format",
            "hair_color: brown, hair_length: short, glasses: True, hat: False, shirt color: red.",
            {
                "hair_color": "brown",
                "hair_length": "short",
                "glasses": True,
                "hat": False,
                "shirt color": "red",
            },
        ),
        (
            "verbose sentence answer",
            "The person has long blonde hair. They are not wearing glasses or a hat. Their shirt color is blue.",
            # parser will likely fail on hair_color/hair_length here — shows the fragility
            {},  # we just print, don't assert
        ),
        (
            "uppercase values",
            "hair_color: Black, hair_length: Long, glasses: YES, hat: NO, shirt color: Green.",
            {
                "hair_color": "black",
                "hair_length": "long",
                "glasses": True,
                "hat": False,
                "shirt color": "green",
            },
        ),
        (
            "missing attributes",
            "hair_color: red, glasses: False.",
            {
                "hair_color": "red",
                "glasses": False,
                "hair_length": "unknown",
                "hat": "unknown",
                "shirt color": "unknown",
            },
        ),
    ]

    print("=== Parser Unit Tests ===\n")
    for desc, raw, expected in cases:
        result = parse_vlm_response(raw, attributes)
        print(f"[{desc}]")
        print(f"  Input   : {raw!r}")
        print(f"  Parsed  : {result}")
        if expected:
            passed = all(result.get(k) == v for k, v in expected.items())
            print(f"  Status  : {'PASS' if passed else 'FAIL'}")
            if not passed:
                for k, v in expected.items():
                    if result.get(k) != v:
                        print(
                            f"    MISMATCH {k!r}: got {result.get(k)!r}, expected {v!r}"
                        )
        print()


# ─── Layer 2: end-to-end ROS service call ─────────────────────────────────────


def test_service(image_path: str):
    import cv2
    import rclpy
    from rclpy.node import Node
    from cv_bridge import CvBridge
    from lasr_vlm_interfaces.srv import VlmDescribePeople

    class VlmTestClient(Node):
        def __init__(self):
            super().__init__("vlm_test_client")
            self.bridge = CvBridge()
            self.client = self.create_client(VlmDescribePeople, "/vlm/describe_people")

        def run(self, image_path: str):
            self.get_logger().info("Waiting for /vlm/describe_people...")
            if not self.client.wait_for_service(timeout_sec=15.0):
                self.get_logger().error("Service not available. Is the server running?")
                return

            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f"Could not read image: {image_path}")
                return

            self.get_logger().info(f"Image loaded: {cv_image.shape} from {image_path}")

            request = VlmDescribePeople.Request()
            request.image_raw = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            self.get_logger().info(
                "Sending request (Ollama inference may take ~10-30s)..."
            )
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

            if future.result() is None:
                self.get_logger().error("Call timed out or returned None.")
                return

            r = future.result()
            print("\n=== Service Response ===")
            print(f"  hair_color  : {r.hair_color}")
            print(f"  hair_length : {r.hair_length}")
            print(f"  glasses     : {r.glasses}")
            print(f"  hat         : {r.hat}")
            print(f"  shirt_color : {r.shirt_color}")
            print("========================\n")

            # Flag unparsed fields
            unknowns = [
                f
                for f, v in [
                    ("hair_color", r.hair_color),
                    ("hair_length", r.hair_length),
                    ("shirt_color", r.shirt_color),
                ]
                if v == "unknown"
            ]
            if unknowns:
                print(
                    f"⚠ These came back 'unknown' — check the raw VLM output in the service logs: {unknowns}"
                )

    rclpy.init()
    node = VlmTestClient()
    try:
        node.run(image_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--unit", action="store_true", help="Run parser unit tests")
    parser.add_argument(
        "--e2e", metavar="IMAGE", help="Run end-to-end service test with this image"
    )
    args = parser.parse_args()

    if not args.unit and not args.e2e:
        parser.print_help()
        sys.exit(1)

    if args.unit:
        test_parser()

    if args.e2e:
        test_service(args.e2e)
