from HRI.states.guide_guest import GuideGuestToLivingRoom
import rclpy
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    coordinates = (8.64, -1.6, 0.0, 0.0)
    sofa = Pose()
    sofa.position.x = coordinates[0]
    sofa.position.y = coordinates[1]
    sofa.position.z = coordinates[2]
    sofa.orientation.w = 1.0

    rclpy.init()
    node = rclpy.create_node("state_machine_example")

    sm = GuideGuestToLivingRoom(node, 1, sofa)

    node.get_logger().info("Hello world")
    outcome = sm.execute()
    rclpy.shutdown()
