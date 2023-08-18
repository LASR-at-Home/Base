# aruco_service

Package for generation of table positions using arUco markers.

This package is maintained by:
- [Peter Tisnikar](mailto:peter.tisnikar@kcl.ac.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- aruco_msgs (build)
- geometry_msgs (build)
- roscpp (build)
- rospy (build)
- std_msgs (build)
- message_generation (build)
- aruco_msgs (exec)
- geometry_msgs (exec)
- roscpp (exec)
- rospy (exec)
- std_msgs (exec)
- message_runtime (exec)



## Usage

This package contains 2 services:
	- Service generating table cuboids (GenerateTableCuboid.py): This service requires an arUco marker placed on the edge of the table, counter, or waiting area.
	- Service publishing points of the table to a topic (publish_points_of_table.py): This service is used for visualising the points set by the table cuboid service.
	
To run the table cuboid service:
	- First, run the arUco marker recognition service with the custom launchfile:
	
	```bash
	rosrun aruco_ros singleTiago.launch
	```
	
	- Then, run the service node:

	```bash
	rosrun aruco_service GenerateTableCuboid.py
	```

	- Finally, you can call the service using the following command: 

	```bash
	rosservice call /generate_table_cuboid table: "N"
	``` 
	where you replace N with -2 for the waiting area, -1 for the counter, and 1-Inf for tables.

	- You can visualize the points using RViz with the configuration set up to show all relevant points:

	```bash
	rosrun rviz rviz -d path/to/file/service_debug.rviz
	```
	
To run the points publisher:
	You need to ensure that the rosparam server on the robot contains the table parameters. If so, you can run:

	```bash
	rosrun aruco_service publish_points_of_table.py to publish them
	```

## Example



## Technical Overview



## ROS Definitions

### Messages

This package has no messages.

### Services

#### `TableNumber`

Request

| Field | Type | Description |
|:-:|:-:|---|
| table | int8 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |


### Actions

This package has no actions.
