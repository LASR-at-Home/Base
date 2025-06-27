#!/bin/bash
# --------------------------------------------------------------------
# Overwrite two TIAGo navigation config files with corrected contents:
#  - recovery_behaviors_rgbd.yaml
#  - global_costmap_plugins_rgbd.yaml
# Keeping structure and inline comments for clarity.
# --------------------------------------------------------------------

set -e

# ----------------------------
# 1. Overwrite recovery_behaviors_rgbd.yaml
# ----------------------------

sudo tee "/opt/pal/gallium/share/pal_navigation_cfg_tiago/config/base/common/recovery_behaviors_rgbd.yaml" > /dev/null <<EOF
recovery_behaviors:
  - name: 'costmap_reset_far'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  - name: 'costmap_reset_close'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
#  - name: 'rotate_recovery'
#    type: 'rotate_recovery/RotateRecovery'

costmap_reset_far:
  reset_distance: 3.0
  layer_names: ["obstacle_laser_layer", "obstacle_rgbd_layer"]

costmap_reset_close:
  reset_distance: 0.5  # changed from 1.5
  layer_names: ["obstacle_laser_layer", "obstacle_rgbd_layer"]

#rotate_recovery:
#  sim_granularity: 0.017
#  frequency: 20.0
#  yaw_goal_tolerance: 0.01
#  max_rotational_vel: 0.25
#  min_in_place_rotational_vel: 0.1
EOF

echo "[INFO] Successfully replaced recovery_behaviors_rgbd.yaml"

# ----------------------------
# 2. Overwrite global_costmap_plugins_rgbd.yaml
# ----------------------------

sudo tee "/opt/pal/gallium/share/pal_navigation_cfg_tiago/config/base/common/global_costmap_plugins_rgbd.yaml" > /dev/null <<EOF
# Independent settings for the planner's costmap
global_costmap:
  # attention, the order and combination method matters
  plugins:
    - name: static_layer
      type: 'costmap_2d::StaticLayer'
    - name: obstacle_laser_layer
      type: 'costmap_2d::ObstacleLayer'
    - name: ramp_layer
      type: 'polycost::PolyCostLayer'
#    - name: static_highway_layer
#      type: 'costmap_2d::StaticLayer'
    - name: obstacle_sonar_layer
      type: 'range_sensor_layer::RangeSensorLayer'
    - name: static_vo_layer
      type: 'costmap_2d::StaticLayer'
    - name: obstacle_rgbd_layer
      type: 'costmap_2d::ObstacleLayer'
    - name: inflation_layer
      type: 'costmap_2d::InflationLayer'
EOF

echo "[INFO] Successfully replaced global_costmap_plugins_rgbd.yaml"

# ----------------------------
# 3. Overwrite move_base.launch
# ----------------------------

sudo tee "/opt/pal/gallium/share/tiago_2dnav/launch/move_base.launch" > /dev/null <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<launch>

  <arg name="config_base_path" default="$(find pal_navigation_cfg_tiago)"/>
  <arg name="global_planner" default="global_planner"/>
  <arg name="local_planner"  default="pal"/>
  <arg name="public_sim"     default="false"/>
  <arg name="rgbd_sensors"   default="false"/>
  <arg name="robot_namespace"      default=""/>
  <arg name="multiple"       default="false"/>
  <arg name="base_type"     default="pmb2"/>

  <!-- Navigation -->
  <include file="$(find pal_navigation_cfg_tiago)/launch/move_base.launch">
    <arg name="rgbd_sensors" value="$(arg rgbd_sensors)"/>
    <arg name="config_base_path" value="$(arg config_base_path)"/>
    <arg name="global_planner" value="$(arg global_planner)"/>
    <arg name="local_planner"  value="$(arg local_planner)"/>
    <arg name="public_sim"     value="$(arg public_sim)"/>
    <arg name="robot_namespace"      value="$(arg robot_namespace)"/>
    <arg name="multiple"       value="$(arg multiple)"/>
    <arg name="base_type"       value="$(arg base_type)"/>
  </include>

</launch>
EOF

echo "[INFO] Successfully replaced move_base.launch"


echo "[DONE] All config files replaced. You can now restart: roslaunch tiago_2dnav move_base.launch"
