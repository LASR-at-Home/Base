#!/bin/bash
# --------------------------------------------------------------------
# Overwrite two TIAGo navigation config files with corrected contents:
#  - recovery_behaviors_rgbd.yaml
#  - global_costmap_plugins_rgbd.yaml
# Keeping structure and inline comments for clarity.
# --------------------------------------------------------------------

set -e

# ----------------------------
# Overwrite recovery_behaviors_rgbd.yaml
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
# Overwrite global_costmap_plugins_rgbd.yaml
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
# Overwrite local_costmap_plugins_rgbd.yaml
# ----------------------------
sudo tee "/opt/pal/gallium/share/pal_navigation_cfg_tiago/config/base/common/local_costmap_plugins_rgbd.yaml" > /dev/null <<'EOF'
# Independent settings for the local costmap
local_costmap:
  # attention, the order and combination method matters
  plugins:
    - name: obstacle_laser_layer
      type: 'costmap_2d::ObstacleLayer'
    - name: ramp_layer
      type: 'polycost::PolyCostLayer'
    - name: obstacle_sonar_layer
      type: 'range_sensor_layer::RangeSensorLayer'
    - name: static_vo_layer
      type: 'costmap_2d::StaticLayer'
    - name: obstacle_rgbd_layer
      type: 'costmap_2d::ObstacleLayer'
    - name: inflation_layer
      type: 'costmap_2d::InflationLayer'
EOF

# ----------------------------
# Overwrite global_costmap.yaml
# ----------------------------

sudo tee "/opt/pal/gallium/share/pal_navigation_cfg_tiago/config/base/common/global_costmap.yaml" > /dev/null <<EOF
# Independent settings for the planner's costmap
global_costmap:
  global_frame    : map
  robot_base_frame: base_footprint
  transform_tolerance: 0.3

  update_frequency : 10.0
  publish_frequency: 1.0

  track_unknown_space: true
  unknown_cost_value : 255

  robot_radius: 0.275

  #plugins are loaded in a separate yaml file

  static_layer:
    enabled  : true
    map_topic: map

  obstacle_laser_layer:
    enabled: true
    observation_sources: base_scan
    combination_method: 0 # can erase static layer

    base_scan:
      sensor_frame: base_laser_link
      data_type: LaserScan
      topic: scan
      expected_update_rate: 0.3
      observation_persistence: 0.0
      inf_is_valid: true
      marking: true
      clearing: true
      raytrace_range: 4.0
      obstacle_range: 3.0

  obstacle_sonar_layer:
    enabled: false
    ns: ""
    topics: ["/sonar_base"]
    no_readings_timeout: 0.0
    clear_threshold: 0.20
    mark_threshold: 0.80
    clear_on_max_reading: true

  ramp_layer:
    enabled : true
    fill_polygons: true
    enable_invert: false
    keep_free: true

  static_highway_layer:
    enabled  : true
    map_topic: highways_map
    first_map_only: false           # subscribes to the map_topic for receiving updates
    trinary_costmap: false
    use_maximum: true               # do not override the previous layers of the costmap

  static_vo_layer:
    enabled  : true
    map_topic: vo_map
    first_map_only: false           # subscribes to the map_topic for receiving updates
    use_maximum: true               # do not override the previous layers of the costmap

  obstacle_rgbd_layer:
    enabled: true
    observation_sources: rgbd_scan
    combination_method: 0
    rgbd_scan:
      sensor_frame: base_footprint
      data_type: LaserScan
      topic: rgbd_scan
      expected_update_rate: 0.3
      observation_persistence: 10.0
      inf_is_valid: true
      marking: true
      clearing: true
      raytrace_range: 4.0
      obstacle_range: 3.0
      blanking_range: 0.1
      mark_blanking: false
      debug: true

  inflation_layer:
    enabled            : true
    inflation_radius   : 0.6
    cost_scaling_factor: 25.0
EOF

echo "[INFO] Successfully replaced global_costmap.yaml"

# ----------------------------
# Overwrite local_costmap.launch
# ----------------------------
sudo tee "/opt/pal/gallium/share/pal_navigation_cfg_tiago/config/base/common/local_costmap.yaml" > /dev/null <<'EOF'
# Independent settings for the local costmap
local_costmap:
  global_frame    : odom
  robot_base_frame: base_footprint
  transform_tolerance: 0.3

  update_frequency : 10.0
  publish_frequency: 1.0

  rolling_window: true
  width         : 4.0
  height        : 4.0
  resolution    : 0.025

  robot_radius: 0.275

  #plugins are loaded in a separate yaml file


  obstacle_laser_layer:
    enabled: true
    observation_sources: base_scan
    combination_method: 0 # can erase static layer

    base_scan:
      sensor_frame: base_laser_link
      data_type: LaserScan
      topic: scan
      expected_update_rate: 0.3
      observation_persistence: 1.0
      inf_is_valid: true
      marking: true
      clearing: true
      raytrace_range: 4.0
      obstacle_range: 3.0

  obstacle_sonar_layer:
    enabled: false
    ns: ""
    topics: ["/sonar_base"]
    no_readings_timeout: 0.0
    clear_threshold: 0.20
    mark_threshold: 0.80
    clear_on_max_reading: true

  ramp_layer:
    enabled : true
    fill_polygons: true
    enable_invert: true

  static_vo_layer:
    enabled  : true
    map_topic: vo_map
    first_map_only: false
    use_maximum: true
    track_unknown_space: false

  obstacle_rgbd_layer:
    enabled: true
    observation_sources: rgbd_scan
    combination_method: 0
    rgbd_scan:
      sensor_frame: base_footprint
      data_type: LaserScan
      topic: rgbd_scan
      expected_update_rate: 0.5
      observation_persistence: 4.0
      inf_is_valid: true
      marking: true
      clearing: true
      raytrace_range: 3.5
      obstacle_range: 3.0
      blanking_range: 0.0
      mark_blanking: false
      debug: false

  inflation_layer:
    enabled            : false # not used for pal_local_planner
    inflation_radius   : 0.55
    cost_scaling_factor: 25.0
EOF

echo "[INFO] Successfully replaced local_costmap.yaml"

# ----------------------------
# Overwrite move_base.launch
# ----------------------------
sudo tee "/opt/pal/gallium/share/tiago_2dnav/launch/move_base.launch" > /dev/null <<'EOF'
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
