#!/usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from lasr_person_tracking_filter.srv import PersonTrackingFilter, PersonTrackingFilterResponse


class PersonTrackingKalmanFilterService:
    """ROS Service wrapper for person tracking Kalman filter"""

    def __init__(self):
        # Initialize 4-state Kalman filter: [x, y, vx, vy]
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State transition matrix (constant velocity model)
        # x_k+1 = x_k + vx_k * dt
        # y_k+1 = y_k + vy_k * dt
        # vx_k+1 = vx_k
        # vy_k+1 = vy_k
        self.kf.F = np.array([[1., 0., 1., 0.],
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

        # Measurement function (observe position only)
        self.kf.H = np.array([[1., 0., 0., 0.],
                              [0., 1., 0., 0.]])

        # Measurement noise covariance (will be adjusted based on detection quality)
        self.kf.R = np.eye(2) * 0.1  # Default: 10cm standard deviation

        # Process noise covariance (will be set when dt is known)
        self.kf.Q = np.eye(4) * 0.01

        # Initial state covariance
        self.kf.P = np.eye(4) * 1000.  # High initial uncertainty

        # State tracking
        self.initialized = False
        self.last_update_time = None
        self.last_prediction_time = None

        # Initialize ROS service
        self.service = rospy.Service(
            '/person_tracking_kalman_filter',
            PersonTrackingFilter,
            self.handle_filter_request
        )

        rospy.loginfo("Person tracking Kalman filter service started")

    def _reset_filter(self):
        """Reset filter to zero state"""
        try:
            # Reset state vector to zeros
            self.kf.x = np.zeros(4)

            # Reset state covariance to high initial uncertainty
            self.kf.P = np.eye(4) * 1000.

            # Reset filter status
            self.initialized = False
            self.last_update_time = None
            self.last_prediction_time = None

            rospy.loginfo("Kalman filter reset to zero state")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to reset Kalman filter: {e}")
            return False

    def handle_filter_request(self, req):
        """Handle incoming service requests"""
        response = PersonTrackingFilterResponse()

        try:
            if req.command == "initialize":
                success = self._initialize(req.x, req.y, req.timestamp)
                response.success = success
                response.message = "Filter initialized" if success else "Initialization failed"

            elif req.command == "update":
                success = self._update_with_detection(
                    req.x, req.y, req.timestamp, req.detection_quality
                )
                response.success = success
                response.message = "Filter updated" if success else "Update failed"

            elif req.command == "predict":
                success = self._predict_forward(req.timestamp)
                response.success = success
                response.message = "Prediction updated" if success else "Prediction failed"

            elif req.command == "get_state":
                response.success = True
                response.message = "State retrieved"

            elif req.command == "is_reasonable":
                is_reasonable = self._is_detection_reasonable(req.x, req.y, req.max_deviation)
                response.success = True
                response.is_reasonable = is_reasonable
                response.message = f"Detection reasonableness: {is_reasonable}"

            elif req.command == "reset":
                success = self._reset_filter()
                response.success = success
                response.message = "Filter reset to zero state" if success else "Reset failed"

            else:
                response.success = False
                response.message = f"Unknown command: {req.command}"
                return response

            # Fill in current state information
            self._fill_state_response(response)

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            rospy.logerr(f"Kalman filter service error: {e}")

        return response

    def _initialize(self, x, y, timestamp):
        """Initialize filter with first detection"""
        try:
            self.kf.x = np.array([x, y, 0., 0.])  # Initial velocity = 0
            self.initialized = True
            self.last_update_time = timestamp
            self.last_prediction_time = timestamp
            rospy.loginfo(f"Kalman filter initialized at position ({x:.2f}, {y:.2f})")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize Kalman filter: {e}")
            return False

    def _update_with_detection(self, x, y, timestamp, detection_quality=1.0):
        """Update filter with new detection"""
        try:
            if not self.initialized:
                return self._initialize(x, y, timestamp)

            # Calculate time step
            dt = (timestamp - self.last_update_time).to_sec()
            if dt <= 0:
                dt = 0.1  # Fallback

            # Update state transition matrix with actual dt
            self.kf.F[0, 2] = dt
            self.kf.F[1, 3] = dt

            # Update process noise based on dt
            # Assume max acceleration of 2 m/s^2
            self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=2.0, block_size=2)

            # Adjust measurement noise based on detection quality
            # Lower quality = higher noise
            measurement_noise = 0.05 / max(detection_quality, 0.1)  # 5cm to 50cm
            self.kf.R = np.eye(2) * (measurement_noise ** 2)

            # Predict step
            self.kf.predict()

            # Update step with measurement
            z = np.array([x, y])
            self.kf.update(z)

            self.last_update_time = timestamp
            self.last_prediction_time = timestamp

            return True

        except Exception as e:
            rospy.logerr(f"Failed to update Kalman filter: {e}")
            return False

    def _predict_forward(self, timestamp):
        """Predict state forward without measurement"""
        try:
            if not self.initialized:
                return False

            # Calculate time step from last prediction
            dt = (timestamp - self.last_prediction_time).to_sec()
            if dt <= 0:
                return True  # No time passed, nothing to predict

            # Update state transition matrix
            self.kf.F[0, 2] = dt
            self.kf.F[1, 3] = dt

            # Update process noise
            self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=2.0, block_size=2)

            # Predict only
            self.kf.predict()

            self.last_prediction_time = timestamp

            return True

        except Exception as e:
            rospy.logerr(f"Failed to predict Kalman filter state: {e}")
            return False

    def _is_detection_reasonable(self, x, y, max_deviation=2.0):
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

            # Flatten position covariance matrix (2x2 -> 4 elements)
            pos_cov = self.kf.P[:2, :2]
            response.position_covariance = [
                float(pos_cov[0, 0]), float(pos_cov[0, 1]),
                float(pos_cov[1, 0]), float(pos_cov[1, 1])
            ]
        else:
            response.position_x = 0.0
            response.position_y = 0.0
            response.velocity_x = 0.0
            response.velocity_y = 0.0
            response.speed = 0.0
            response.position_covariance = [0.0, 0.0, 0.0, 0.0]


def main():
    rospy.init_node('person_tracking_kalman_filter_service')

    try:
        filter_service = PersonTrackingKalmanFilterService()
        rospy.loginfo("Kalman filter service is ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Kalman filter service shutdown")


if __name__ == '__main__':
    main()
