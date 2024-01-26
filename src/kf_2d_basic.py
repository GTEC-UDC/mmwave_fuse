import numpy as np

class KalmanFilter:
    def __init__(self, max_velocity, time_step, initial_measurement=None):
        self.max_velocity = max_velocity
        self.dt = time_step  # Time step

        # State variables
        self.x = np.zeros((4, 1))  # [x, y, vx, vy]
        self.P = np.eye(4)  # Error covariance matrix

        # Measurement matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Measurement noise covariance
        self.R = np.eye(2)

        # Process noise covariance
        self.Q = np.eye(4)

        if initial_measurement is not None:
            x, y = initial_measurement
            self.x[:2] = np.array([x, y]).reshape((2, 1))
            
    def predict(self):
        # Predict the next state based on the current state and control input
        self.x = self.F.dot(self.x)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q

    def update(self, measurement=None):
        # Update the state based on the measurement
        if measurement is not None:
            x, y, error_est = measurement

            # Measurement noise covariance
            self.R = np.eye(2) * error_est**2

            # Measurement update
            Z = np.array([x, y]).reshape((2, 1))
            y = Z - self.H.dot(self.x)
            S = self.H.dot(self.P).dot(self.H.T) + self.R
            K = self.P.dot(self.H.T).dot(np.linalg.inv(S))

            self.x = self.x + K.dot(y)
            self.P = (np.eye(4) - K.dot(self.H)).dot(self.P)

        # Velocity saturation to limit maximum velocity
        self.x[2] = np.clip(self.x[2], -self.max_velocity, self.max_velocity)
        self.x[3] = np.clip(self.x[3], -self.max_velocity, self.max_velocity)

        # State transition matrix
        self.F = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        # Predict the next state
        self.predict()

        # Return the estimated position and covariance error matrix
        return self.x[0, 0], self.x[1, 0], self.P


# # Example usage
# max_velocity = 5  # Maximum velocity of the target in meters per second
# initial_coordinate = (0, 0)  # Initial coordinate
# kalman_filter = KalmanFilter(max_velocity, initial_measurement=initial_coordinate)

# measurements = [(1, 1, 0.1), (2, 2, 0.2), (3, 3, 0.3), (4, 4, 0.4), (5, 5, 0.5)]

# for measurement in measurements:
#     x, y, error = measurement
#     estimated_x, estimated_y, cov_matrix = kalman_filter.update([x, y])
#     print(f"Measured: ({x}, {y}), Estimated: ({estimated_x:.2f}, {estimated_y:.2f}), Covariance: {cov_matrix}")

# estimated_x, estimated_y, cov_matrix = kalman_filter.update()
# print(f"No measurement, Estimated: ({estimated_x:.2f}, {estimated_y:.2f}), Covariance: {cov_matrix}")

