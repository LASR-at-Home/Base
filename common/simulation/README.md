# simulation

TIAGo (public simulation) in the `house` world, with MoveIt and object grasping.

## Launch

```bash
ros2 launch simulation sim.launch.py   # Gazebo + MoveIt
ros2 launch simulation nav.launch.py   # the same plus Nav2 and RViz (use this if using yolo)
```

Arguments (both launch files):

- `rviz:=True` opens RViz with MoveIt's MotionPlanning display (with `nav.launch.py`, next to its mapping RViz)
- `gzclient:=False` runs Gazebo without its GUI
- `world:=robocup_map` defines which world map to use. (default "robocup_map". Old map is "house")

## Grasping

Both launch files start PAL's `move_group` with MoveIt Task Constructor's
`ExecuteTaskSolutionCapability`, so arm and gripper motions can be planned and
executed from RViz or with MTC.

Gazebo can't hold objects in TIAGo's gripper by friction (the fingers are
position controlled and sink into objects), so `worlds/house.world` loads the
`grasp_assist` world plugin (`src/grasp_assist.cpp`). When both fingers touch an
object for 20 ms, the object is attached to the gripper; it is released when the
fingers open again. Any non-static model up to `<max_mass>` (2 kg) can be
grasped, including models spawned at runtime; add `<object>name</object>`
entries to the plugin to restrict it to specific models. Its messages go to
`~/.gazebo/server-11345/default.log`.

Close the fingers to about 5 mm inside the object's surface rather than fully:
they stop at the object, and with a larger error the gripper controller (2 cm
goal tolerance) may report the motion as failed.

## Mapping
1. Launch the simulation using:
    ```
    ros2 launch simulation sim.launch.py
    ``` 
    or launch a fresh gazebo world with TIAGo


2. Mapping:
    ```
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file <path_to_slam_params.yaml>
    ```

3. Save the map:

    ```
    ros2 run nav2_map_server map_saver_cli -f <path_map>
    ```