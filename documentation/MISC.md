#### Misc Documentation for Porting to ROS2

##### rospy.wait_for_message()
Need to create a subscription and check if message is received

##### rospy.get_param()
Use rclpy's `node.get_parameter()` instead. Returns a Parameter object, so value needs to be accessed with:
`node.get_parameter('param_name').get_parameter_value()` and the correct type, e.g. `.string_value`
or use `node.get_parameter('param_name').value`
See [Play Motion skill for an example](../skills/src/lasr_skills/play_motion.py)

##### rospkg
`package_root = rospkg.RosPack().get_path("PACKAGE_NAME")"` can be replaced with:
``` python
from ament_index_python import packages

package_install = packages.get_package_prefix("PACKAGE_NAME")
package_root = os.path.abspath(os.path.join(package_install, os.pardir, os.pardir, "PACKAGE_NAME",))
```
> Pay attention to the path to root from the current script, and adjust accordingly
 
##### Launch files
- [ros2 launch files guide](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/)