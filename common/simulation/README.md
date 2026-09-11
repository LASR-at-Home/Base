

Launch: 
 ros2 launch simulation nav.launch.py if using yolo 
 ros2 launch simulation sim.launch.py for others


slam_toolbox / neccesary dependencies may not be availiable

Mapping (): 
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file <path_to_slam_params.yaml>

Save the map: 
    ros2 run nav2_map_server map_saver_cli -f <path_map>

