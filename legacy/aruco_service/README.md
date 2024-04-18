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

This package contains 3 services:
	- Service generating table cuboids (GenerateTableCuboid.py): This service requires an arUco marker placed on the edge of the table, counter, or waiting area.
	- Service publishing points of the table to a topic (publish_points_of_table.py): This service is used for visualising the points set by the table cuboid service.
	- Service storing the robot positions at the table (save_navigation_points.py): This service stores the current position of the robot into the location namespace of the coffee shop representation.
	
To run the table cuboid service:
	- First, run the arUco marker recognition service with the custom launch file:
	
	```bash
	roslaunch aruco_ros singleTiago.launch
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
	rosrun rviz rviz -d $(rospack find aruco_service)/service_debug.rviz
	```
	
To run the points publisher:
	You need to ensure that the rosparam server on the robot contains the table parameters. If so, you can run:

	```bash
	rosrun aruco_service publish_points_of_table.py
	```
	You can then call the service with the following command:

	```bash
	rosservice call /publish_table_points table: "N"
	``` 
	Where you replace N with -2 for the waiting area, -1 for the counter, and 1-Inf for tables.

To run the navigation points storing service:
	You need to ensure that the rosparam server on the robot contains the table parameters. If so, you can run:

	```bash
	rosrun aruco_service save_navigation_points.py
	```
	You can then call the service with the following command:

	```bash
	rosservice call /save_navigation_points table: "N"
	``` 
	Where you replace N with -2 for the waiting area, -1 for the counter, and 1-Inf for tables.

## Example

Running the service

TODOTODOTODO

## Technical Overview

TODO

## ROS Definitions

### Launch Files

#### `singleTiago`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| markerId | 500 |  |
| markerSize | 0.1 |  |
| eye | left |  |
| marker_frame | aruco_marker_frame |  |
| ref_frame | /map |  |
| corner_refinement | LINES |  |


#### `coffee_shop_prep`

No description provided.


### Messages

This package has no messages.

### Services

#### `SaveNavigationPoint`

Request

| Field | Type | Description |
|:-:|:-:|---|
| table | int8 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |

#### `PublishTablePoints`

Request

| Field | Type | Description |
|:-:|:-:|---|
| table | int8 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |

#### `GenerateTableCuboid`

Request

| Field | Type | Description |
|:-:|:-:|---|
| table | int8 |  |
| long_side | float64 |  |
| short_side | float64 |  |
| padding | float64 |  |
| is_rect | bool |  |
| radius | float64 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |


### Actions

This package has no actions.
