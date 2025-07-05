#!/usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from lasr_person_tracking_filter.srv import (
    DualSensorPersonTrackingFilter,
    DualSensorPersonTrackingFilterResponse,
)


class PersonTrackingDualInputKalmanFilterService:
    """ROS Service wrapper for dual-input person tracking Kalman filter

    Designed for robust tracking with two potentially noisy, asynchronous sensors.
    Supports completely asynchronous sensor inputs that may never be synchronized.
    Focuses on stability over precision, with simple parameter tuning.
    """

    def __init__(self):
        # Initialize 4-state Kalman filter: [x, y, vx, vy]
        self.kf = KalmanFilter(dim_x=4, dim_z=2)  # dim_z will be adjusted dynamically

        # State transition matrix (constant velocity model)
        self.kf.F = np.array(
            [
                [1.0, 0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0, 1.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Measurement function (observe position only)
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

        # Default measurement noise - conservative for rough signals
        self.kf.R = np.eye(2) * 1.25

        # Process noise covariance - conservative for stability
        self.kf.Q = np.eye(4) * 1.25

        # Make velocity states more uncertain
        self.kf.Q[2, 2] *= 3.0  # 3x more uncertainty for vx
        self.kf.Q[3, 3] *= 3.0  # 3x more uncertainty for vy

        # Initial state covariance - high uncertainty for first detection
        self.kf.P = np.eye(4) * 2000.0  # High initial uncertainty

        # Simple thresholds for dual sensor management
        self.agreement_thresholds = {
            "good_agreement": 0.8,  # < 80cm: normal fusion
            "poor_agreement": 1.5,  # 80cm-1.5m: cautious fusion
            "conflict": 2.5,  # > 2.5m: conflict mode
        }

        # Asynchronous sensor buffering - key enhancement for async support
        self.sensor_buffer_window = rospy.Duration.from_sec(
            0.5
        )  # 500ms window for temporal fusion
        self.max_temporal_gap = rospy.Duration.from_sec(
            2.5
        )  # 2s max gap before considering sensor dead

        # Enhanced sensor tracking with temporal information
        self.last_sensor_a = {
            "x": None,
            "y": None,
            "time": None,
            "quality": 0.0,
            "update_count": 0,
            "last_used_time": None,
        }
        self.last_sensor_b = {
            "x": None,
            "y": None,
            "time": None,
            "quality": 0.0,
            "update_count": 0,
            "last_used_time": None,
        }
        self.sensor_reliability = {"a": 1.0, "b": 1.0}

        # State tracking
        self.initialized = False
        self.last_update_time = None
        self.last_prediction_time = None

        # Fusion state
        self.fusion_mode = "normal"
        self.consecutive_conflicts = 0

        # Asynchronous processing state
        self.last_filter_update_time = None
        self.prediction_mode = False  # Whether we're in prediction-only mode

        # Initialize ROS service
        self.service = rospy.Service(
            "/person_tracking_dual_kalman_filter",
            DualSensorPersonTrackingFilter,
            self.handle_filter_request,
        )

        # Sensor role definitions based on quality - conservative approach
        self.sensor_roles = {
            "a": "secondary",
            "b": "secondary",
        }  # Default both as secondary
        self.primary_sensor_threshold = (
            0.75  # Quality threshold to be considered primary
        )
        self.reliability_decay_rate = (
            0.005  # Conservative decay rate for unused sensors
        )

        rospy.loginfo(
            "Dual-input asynchronous person tracking Kalman filter service started"
        )

    def _reset_filter(self):
        """Reset dual filter to zero state"""
        try:
            # Reset state vector to zeros
            self.kf.x = np.zeros(4)

            # Reset state covariance to high initial uncertainty
            self.kf.P = np.eye(4) * 2000.0

            # Reset filter status
            self.initialized = False
            self.last_update_time = None
            self.last_prediction_time = None
            self.last_filter_update_time = None

            # Reset sensor buffers
            self.last_sensor_a = {
                "x": None,
                "y": None,
                "time": None,
                "quality": 0.0,
                "update_count": 0,
                "last_used_time": None,
            }
            self.last_sensor_b = {
                "x": None,
                "y": None,
                "time": None,
                "quality": 0.0,
                "update_count": 0,
                "last_used_time": None,
            }

            # Reset sensor reliability
            self.sensor_reliability = {"a": 1.0, "b": 1.0}

            # Reset fusion state
            self.fusion_mode = "normal"
            self.consecutive_conflicts = 0
            self.prediction_mode = False

            rospy.loginfo("Dual Kalman filter reset to zero state")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to reset dual Kalman filter: {e}")
            return False

    def handle_filter_request(self, req):
        """Handle incoming service requests with full asynchronous support"""
        response = DualSensorPersonTrackingFilterResponse()

        try:
            if req.command == "initialize":
                # Support both old and new initialization methods
                init_x = getattr(req, "sensor_a_x", None) or getattr(req, "x", 0.0)
                init_y = getattr(req, "sensor_a_y", None) or getattr(req, "y", 0.0)
                success = self._initialize(init_x, init_y, req.timestamp)
                response.success = success
                response.message = (
                    "Filter initialized" if success else "Initialization failed"
                )

            elif (
                req.command == "update" or req.command == "update_single"
            ):  # Support both commands
                # Handle single sensor update asynchronously
                x = getattr(req, "sensor_a_x", None) or getattr(req, "x", None)
                y = getattr(req, "sensor_a_y", None) or getattr(req, "y", None)
                quality = getattr(req, "sensor_a_quality", None) or getattr(
                    req, "detection_quality", 0.5
                )

                if x is not None and y is not None:
                    success = self._update_with_single_sensor_async(
                        x, y, req.timestamp, quality, "a"
                    )
                else:
                    success = False

                response.success = success
                response.message = (
                    "Single sensor async update" if success else "Single update failed"
                )

            elif req.command == "update_dual":
                # Handle dual sensor input with full async support
                success = self._update_with_dual_sensors_async(req)
                response.success = success
                response.message = (
                    "Dual sensor async update" if success else "Dual update failed"
                )

            elif req.command == "predict":
                success = self._predict_forward(req.timestamp)
                response.success = success
                response.message = (
                    "Prediction updated" if success else "Prediction failed"
                )

            elif req.command == "get_state":
                # Always predict to current time before returning state
                self._predict_to_time(req.timestamp)
                response.success = True
                response.message = "State retrieved (predicted to current time)"

            elif (
                req.command == "is_reasonable" or req.command == "check_reasonable"
            ):  # Support both commands
                # Predict to current time before checking reasonableness
                self._predict_to_time(req.timestamp)
                x = getattr(req, "sensor_a_x", None) or getattr(req, "x", 0.0)
                y = getattr(req, "sensor_a_y", None) or getattr(req, "y", 0.0)
                max_dev = getattr(req, "max_deviation", 3.0)
                is_reasonable = self._is_detection_reasonable(x, y, max_dev)
                response.success = True
                response.is_reasonable = is_reasonable
                response.message = f"Detection reasonableness: {is_reasonable}"

            elif req.command == "reset":
                success = self._reset_filter()
                response.success = success
                response.message = (
                    "Dual filter reset to zero state" if success else "Reset failed"
                )

            else:
                response.success = False
                response.message = f"Unknown command: {req.command}"
                return response

            # Fill in current state information
            self._fill_state_response(response)

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            rospy.logerr(f"Dual Kalman filter service error: {e}")

        return response

    def _update_with_single_sensor_async(self, x, y, timestamp, quality, sensor_id):
        """Update filter with single sensor input in fully asynchronous mode"""
        try:
            # Update sensor buffer
            if sensor_id == "a":
                self.last_sensor_a.update(
                    {
                        "x": x,
                        "y": y,
                        "time": timestamp,
                        "quality": quality,
                        "update_count": self.last_sensor_a["update_count"] + 1,
                        "last_used_time": timestamp,
                    }
                )
            else:
                self.last_sensor_b.update(
                    {
                        "x": x,
                        "y": y,
                        "time": timestamp,
                        "quality": quality,
                        "update_count": self.last_sensor_b["update_count"] + 1,
                        "last_used_time": timestamp,
                    }
                )

            # Initialize if needed
            if not self.initialized:
                return self._initialize(x, y, timestamp)

            # Process the update asynchronously
            return self._process_asynchronous_update(timestamp)

        except Exception as e:
            rospy.logerr(f"Failed to update with single sensor async: {e}")
            return False

    def _update_with_dual_sensors_async(self, req):
        """Update filter with dual sensor input supporting complete asynchronicity"""
        try:
            current_time = req.timestamp

            # Update sensor buffers based on availability
            if getattr(req, "sensor_a_available", False):
                self.last_sensor_a.update(
                    {
                        "x": getattr(req, "sensor_a_x", None),  # Fixed: was req.x
                        "y": getattr(req, "sensor_a_y", None),  # Fixed: was req.y
                        "time": current_time,
                        "quality": getattr(
                            req, "sensor_a_quality", 0.5
                        ),  # Fixed: was detection_quality
                        "update_count": self.last_sensor_a["update_count"] + 1,
                        "last_used_time": current_time,
                    }
                )

            if getattr(req, "sensor_b_available", False):
                self.last_sensor_b.update(
                    {
                        "x": getattr(
                            req, "sensor_b_x", None
                        ),  # This was already correct
                        "y": getattr(
                            req, "sensor_b_y", None
                        ),  # This was already correct
                        "time": current_time,
                        "quality": getattr(
                            req, "sensor_b_quality", 0.5
                        ),  # This was already correct
                        "update_count": self.last_sensor_b["update_count"] + 1,
                        "last_used_time": current_time,
                    }
                )

            # Initialize if needed
            if not self.initialized:
                return self._initialize_with_any_available_sensor(current_time)

            # Process asynchronous update
            return self._process_asynchronous_update(current_time)

        except Exception as e:
            rospy.logerr(f"Failed to update with dual sensors async: {e}")
            return False

    def _process_asynchronous_update(self, current_time):
        """Core asynchronous processing logic"""
        try:
            # First, predict filter to current time
            self._predict_to_time(current_time)

            # Determine which sensors are currently viable
            viable_sensors = self._get_viable_sensors(current_time)

            # Analyze and update sensor roles based on recent quality patterns
            self._analyze_sensor_roles(current_time)

            if not viable_sensors:
                # No viable sensors, stay in prediction mode
                self.fusion_mode = "predict_only"
                self.prediction_mode = True
                return True

            # Analyze temporal alignment and sensor agreement
            fusion_strategy = self._analyze_temporal_fusion_strategy(
                viable_sensors, current_time
            )

            # Apply fusion strategy
            if fusion_strategy:
                self._apply_temporal_fusion(fusion_strategy, current_time)
                self.prediction_mode = False
            else:
                # Fallback to prediction only
                self.fusion_mode = "predict_only"
                self.prediction_mode = True

            self.last_filter_update_time = current_time
            return True

        except Exception as e:
            rospy.logerr(f"Failed to process asynchronous update: {e}")
            return False

    def _get_viable_sensors(self, current_time):
        """Determine which sensors are viable for current update"""
        viable = []

        # Check sensor A viability
        if (
            self.last_sensor_a["time"] is not None
            and self.last_sensor_a["x"] is not None
            and (current_time - self.last_sensor_a["time"]) < self.max_temporal_gap
        ):
            viable.append("a")

        # Check sensor B viability
        if (
            self.last_sensor_b["time"] is not None
            and self.last_sensor_b["x"] is not None
            and (current_time - self.last_sensor_b["time"]) < self.max_temporal_gap
        ):
            viable.append("b")

        return viable

    def _analyze_sensor_roles(self, current_time):
        """Analyze and update sensor roles based on quality patterns - conservative approach"""
        # Update sensor roles based on average quality over recent updates
        for sensor_id in ["a", "b"]:
            sensor_data = self.last_sensor_a if sensor_id == "a" else self.last_sensor_b

            if sensor_data["quality"] >= self.primary_sensor_threshold:
                self.sensor_roles[sensor_id] = "primary"
            elif sensor_data["quality"] >= 0.25:  # Medium quality threshold
                self.sensor_roles[sensor_id] = "secondary"
            else:
                self.sensor_roles[sensor_id] = "auxiliary"  # Low quality

    def _analyze_temporal_fusion_strategy(self, viable_sensors, current_time):
        """Analyze fusion strategy considering temporal aspects"""
        if len(viable_sensors) == 0:
            return None

        elif len(viable_sensors) == 1:
            # Single sensor case
            sensor_id = viable_sensors[0]
            sensor_data = self.last_sensor_a if sensor_id == "a" else self.last_sensor_b

            # Check temporal freshness
            age = (current_time - sensor_data["time"]).to_sec()
            temporal_weight = max(0.1, 1.0 - age / self.sensor_buffer_window.to_sec())

            return {
                "type": "single",
                "sensor_id": sensor_id,
                "data": sensor_data,
                "temporal_weight": temporal_weight,
                "age": age,
            }

        else:
            # Dual sensor case - check temporal alignment
            age_a = (current_time - self.last_sensor_a["time"]).to_sec()
            age_b = (current_time - self.last_sensor_b["time"]).to_sec()

            # Calculate temporal weights
            weight_a = max(0.1, 1.0 - age_a / self.sensor_buffer_window.to_sec())
            weight_b = max(0.1, 1.0 - age_b / self.sensor_buffer_window.to_sec())

            # Apply conservative role-based weight adjustments for dual fusion
            if self.sensor_roles["a"] == "primary":
                weight_a *= 1.2  # 20% bonus for primary sensor
            if self.sensor_roles["b"] == "primary":
                weight_b *= 1.2

            # Check spatial agreement (project older measurement to current time if needed)
            projected_a = self._project_measurement_to_time(
                self.last_sensor_a, current_time
            )
            projected_b = self._project_measurement_to_time(
                self.last_sensor_b, current_time
            )

            if projected_a and projected_b:
                distance = self._calculate_projected_distance(projected_a, projected_b)

                # Determine fusion type based on agreement and temporal alignment
                temporal_sync = abs(age_a - age_b) < 0.2  # 200ms sync tolerance

                if (
                    distance < self.agreement_thresholds["good_agreement"]
                    and temporal_sync
                ):
                    fusion_type = "optimal"
                elif distance < self.agreement_thresholds["poor_agreement"]:
                    fusion_type = "cautious"
                else:
                    # Large disagreement - choose better sensor with role-based weighting
                    # Primary sensors get priority, but conservatively
                    role_bonus_a = 1.3 if self.sensor_roles["a"] == "primary" else 1.0
                    role_bonus_b = 1.3 if self.sensor_roles["b"] == "primary" else 1.0

                    score_a = (
                        self.sensor_reliability["a"]
                        * weight_a
                        * self.last_sensor_a["quality"]
                        * role_bonus_a  # Conservative 30% bonus for primary sensors
                    )
                    score_b = (
                        self.sensor_reliability["b"]
                        * weight_b
                        * self.last_sensor_b["quality"]
                        * role_bonus_b
                    )

                    better_sensor = "a" if score_a > score_b else "b"

                    return {
                        "type": "conflict_single",
                        "sensor_id": better_sensor,
                        "data": (
                            self.last_sensor_a
                            if better_sensor == "a"
                            else self.last_sensor_b
                        ),
                        "conflict_distance": distance,
                    }

                return {
                    "type": "dual",
                    "fusion_type": fusion_type,
                    "data_a": projected_a,
                    "data_b": projected_b,
                    "weight_a": weight_a,
                    "weight_b": weight_b,
                    "temporal_sync": temporal_sync,
                    "distance": distance,
                }

        return None

    def _project_measurement_to_time(self, sensor_data, target_time):
        """Project sensor measurement to target time using current velocity estimate"""
        if not self.initialized or sensor_data["time"] is None:
            return sensor_data

        dt = (target_time - sensor_data["time"]).to_sec()

        # Use current velocity estimate for projection
        current_vel = self.kf.x[2:4]

        projected = {
            "x": sensor_data["x"] + current_vel[0] * dt,
            "y": sensor_data["y"] + current_vel[1] * dt,
            "quality": sensor_data["quality"],
            "time": target_time,
            "projected": True,
            "projection_dt": dt,
        }

        return projected

    def _calculate_projected_distance(self, proj_a, proj_b):
        """Calculate distance between projected measurements"""
        dx = proj_a["x"] - proj_b["x"]
        dy = proj_a["y"] - proj_b["y"]
        return np.sqrt(dx * dx + dy * dy)

    def _apply_temporal_fusion(self, strategy, current_time):
        """Apply the determined fusion strategy"""
        try:
            if strategy["type"] == "single" or strategy["type"] == "conflict_single":
                # Single sensor fusion with temporal weighting
                sensor_data = strategy["data"]

                # Adjust noise based on temporal weight and age
                base_noise = 0.2
                temporal_factor = 1.0 / strategy.get("temporal_weight", 0.5)
                age_factor = (
                    1.0 + strategy.get("age", 0) * 0.1
                )  # 10% penalty per second

                noise = (
                    base_noise
                    * temporal_factor
                    * age_factor
                    / max(sensor_data["quality"], 0.1)
                )

                # Use projected position if available
                if sensor_data.get("projected", False):
                    obs_x, obs_y = sensor_data["x"], sensor_data["y"]
                else:
                    obs_x, obs_y = sensor_data["x"], sensor_data["y"]

                # Apply single sensor update
                self.kf.R = np.eye(2) * (noise**2)
                z = np.array([obs_x, obs_y])
                self.kf.update(z)

                self.fusion_mode = f"single_{strategy['sensor_id']}_temporal"

                if strategy["type"] == "conflict_single":
                    self.consecutive_conflicts += 1

            elif strategy["type"] == "dual":
                # Dual sensor fusion with sequential updates - much safer!
                data_a = strategy["data_a"]
                data_b = strategy["data_b"]

                # Calculate noise for each sensor with temporal weighting
                base_noise = 0.15 if strategy["fusion_type"] == "optimal" else 0.3

                noise_a = base_noise / (
                    strategy["weight_a"] * max(data_a["quality"], 0.1)
                )
                noise_b = base_noise / (
                    strategy["weight_b"] * max(data_b["quality"], 0.1)
                )

                # Apply additional penalty for non-synchronized sensors
                if not strategy["temporal_sync"]:
                    noise_a *= 1.5
                    noise_b *= 1.5

                # Validate measurements before updating
                if (
                    data_a["x"] is None
                    or data_a["y"] is None
                    or data_b["x"] is None
                    or data_b["y"] is None
                ):
                    rospy.logwarn("Invalid dual sensor measurements, skipping fusion")
                    return

                # Sequential updates - no dimension changes needed!
                # First update with sensor A
                self.kf.R = np.eye(2) * (noise_a**2)
                z_a = np.array([data_a["x"], data_a["y"]])
                self.kf.update(z_a)

                # Then update with sensor B (using the updated state)
                self.kf.R = np.eye(2) * (noise_b**2)
                z_b = np.array([data_b["x"], data_b["y"]])
                self.kf.update(z_b)

                self.fusion_mode = f"dual_{strategy['fusion_type']}_sequential"

                # Reset consecutive conflicts since we successfully fused
                self.consecutive_conflicts = 0

            # Update sensor reliability with conservative role-based adjustments
            if "sensor_id" in strategy:
                sensor_id = strategy["sensor_id"]
                if strategy["type"] == "single":
                    # Single sensor mode: reduce reliability slightly (conservative approach)
                    self.sensor_reliability[sensor_id] = max(
                        0.1, self.sensor_reliability[sensor_id] - 0.002
                    )
                    # Decay unused sensor more significantly
                    other_sensor = "b" if sensor_id == "a" else "a"
                    self.sensor_reliability[other_sensor] = max(
                        0.1,
                        self.sensor_reliability[other_sensor]
                        - self.reliability_decay_rate,
                    )
                else:  # conflict_single case
                    # Successful conflict resolution: small increase for winner
                    self.sensor_reliability[sensor_id] = min(
                        1.0, self.sensor_reliability[sensor_id] + 0.005
                    )
                    # Larger decrease for rejected sensor
                    other_sensor = "b" if sensor_id == "a" else "a"
                    self.sensor_reliability[other_sensor] = max(
                        0.1, self.sensor_reliability[other_sensor] - 0.02
                    )
            else:
                # Dual sensor success: both sensors gain reliability
                self.sensor_reliability["a"] = min(
                    1.0, self.sensor_reliability["a"] + 0.008
                )
                self.sensor_reliability["b"] = min(
                    1.0, self.sensor_reliability["b"] + 0.008
                )

        except Exception as e:
            rospy.logerr(f"Failed to apply temporal fusion: {e}")
            # Optional: Reset to single sensor mode on failure
            self.fusion_mode = "error_fallback"

    def _predict_to_time(self, target_time):
        """Predict filter state to target time"""
        if not self.initialized:
            return False

        if self.last_filter_update_time is None:
            self.last_filter_update_time = self.last_update_time or target_time

        dt = (target_time - self.last_filter_update_time).to_sec()

        if dt > 0:
            # Update state transition matrix
            self.kf.F[0, 2] = dt
            self.kf.F[1, 3] = dt

            # Update process noise
            self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=1.5, block_size=2)

            # Predict
            self.kf.predict()

            self.last_filter_update_time = target_time

        return True

    def _initialize_with_any_available_sensor(self, timestamp):
        """Initialize with any available sensor"""
        # Try sensor A first
        if (
            self.last_sensor_a["x"] is not None
            and self.last_sensor_a["time"] is not None
        ):
            return self._initialize(
                self.last_sensor_a["x"], self.last_sensor_a["y"], timestamp
            )

        # Try sensor B
        elif (
            self.last_sensor_b["x"] is not None
            and self.last_sensor_b["time"] is not None
        ):
            return self._initialize(
                self.last_sensor_b["x"], self.last_sensor_b["y"], timestamp
            )

        return False

    # Keep all existing helper methods unchanged
    def _initialize(self, x, y, timestamp):
        """Initialize filter with first detection"""
        try:
            self.kf.x = np.array([x, y, 0.0, 0.0])  # Initial velocity = 0
            self.initialized = True
            self.last_update_time = timestamp
            self.last_prediction_time = timestamp
            self.last_filter_update_time = timestamp
            self.consecutive_conflicts = 0
            self.prediction_mode = False
            rospy.loginfo(
                f"Dual Kalman filter initialized at position ({x:.2f}, {y:.2f})"
            )
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize dual Kalman filter: {e}")
            return False

    def _predict_forward(self, timestamp):
        """Predict state forward without measurement"""
        try:
            return self._predict_to_time(timestamp)
        except Exception as e:
            rospy.logerr(f"Failed to predict dual Kalman filter state: {e}")
            return False

    def _is_detection_reasonable(self, x, y, max_deviation=3.0):
        """Check if detection is reasonable compared to prediction"""
        if not self.initialized:
            return True

        predicted_pos = self.kf.x[:2]
        distance = np.sqrt((x - predicted_pos[0]) ** 2 + (y - predicted_pos[1]) ** 2)
        return distance < max_deviation

    def _fill_state_response(self, response):
        """Fill response with current state information"""
        response.initialized = self.initialized

        if self.initialized:
            response.position_x = float(self.kf.x[0])
            response.position_y = float(self.kf.x[1])
            response.velocity_x = float(self.kf.x[2])
            response.velocity_y = float(self.kf.x[3])
            response.speed = float(np.sqrt(self.kf.x[2] ** 2 + self.kf.x[3] ** 2))

            # Flatten position covariance matrix
            pos_cov = self.kf.P[:2, :2]
            response.position_covariance = [
                float(pos_cov[0, 0]),
                float(pos_cov[0, 1]),
                float(pos_cov[1, 0]),
                float(pos_cov[1, 1]),
            ]

            # Fill dual sensor specific fields
            response.fusion_mode = self.fusion_mode
            response.sensor_a_reliability = float(self.sensor_reliability["a"])
            response.sensor_b_reliability = float(self.sensor_reliability["b"])
            response.consecutive_conflicts = int(self.consecutive_conflicts)

            # Calculate sensor agreement distance if both sensors are available
            if (
                self.last_sensor_a["x"] is not None
                and self.last_sensor_b["x"] is not None
            ):
                dx = self.last_sensor_a["x"] - self.last_sensor_b["x"]
                dy = self.last_sensor_a["y"] - self.last_sensor_b["y"]
                response.sensor_agreement_distance = float(np.sqrt(dx * dx + dy * dy))
            else:
                response.sensor_agreement_distance = -1.0

            # Set conflict detection flag
            response.conflict_detected = self.consecutive_conflicts > 0

            # Calculate confidence level based on fusion mode and sensor reliability
            if self.fusion_mode.startswith("dual"):
                response.confidence_level = min(
                    self.sensor_reliability["a"], self.sensor_reliability["b"]
                )
            elif "single_a" in self.fusion_mode:
                response.confidence_level = self.sensor_reliability["a"] * 0.8
            elif "single_b" in self.fusion_mode:
                response.confidence_level = self.sensor_reliability["b"] * 0.8
            else:
                response.confidence_level = 0.3  # prediction only

            # Set system health
            if self.fusion_mode.startswith("dual"):
                response.system_health = "healthy"
            elif "single" in self.fusion_mode:
                response.system_health = "warning"
            elif "predict_only" in self.fusion_mode:
                response.system_health = "degraded"
            else:
                response.system_health = "healthy"

        else:
            response.position_x = 0.0
            response.position_y = 0.0
            response.velocity_x = 0.0
            response.velocity_y = 0.0
            response.speed = 0.0
            response.position_covariance = [0.0, 0.0, 0.0, 0.0]
            response.fusion_mode = "uninitialized"
            response.sensor_a_reliability = 1.0
            response.sensor_b_reliability = 1.0
            response.consecutive_conflicts = 0
            response.sensor_agreement_distance = -1.0
            response.conflict_detected = False
            response.confidence_level = 0.0
            response.system_health = "uninitialized"


def main():
    rospy.init_node("person_tracking_dual_kalman_filter_service")

    try:
        filter_service = PersonTrackingDualInputKalmanFilterService()
        rospy.loginfo("Dual-input asynchronous Kalman filter service is ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dual-input asynchronous Kalman filter service shutdown")


if __name__ == "__main__":
    main()
