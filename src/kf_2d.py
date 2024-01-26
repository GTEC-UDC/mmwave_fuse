import numpy as np

class KalmanFilter2D:
    def __init__(self, max_velocity, initial_position, measurement_error, dt):
        self.max_velocity = max_velocity
        self.dt = dt

        # Inicialización de las matrices de estado y covarianza
        self.state = np.array([initial_position[0], 0, initial_position[1], 0], dtype=float)
        self.covariance = np.zeros((4, 4), dtype=float)

        # Inicialización de la matriz de transición de estado
        self.transition_matrix = np.array([[1, dt, 0, 0],
                                           [0, 1, 0, 0],
                                           [0, 0, 1, dt],
                                           [0, 0, 0, 1]], dtype=float)

        # Inicialización de la matriz de medición
        self.measurement_matrix = np.array([[1, 0, 0, 0],
                                            [0, 0, 1, 0]], dtype=float)

        # Inicialización de la matriz de ruido de proceso
        self.process_noise_cov = np.array([[dt**4/4, dt**3/2, 0, 0],
                                           [dt**3/2, dt**2, 0, 0],
                                           [0, 0, dt**4/4, dt**3/2],
                                           [0, 0, dt**3/2, dt**2]], dtype=float)

        # Inicialización de la matriz de ruido de medición
        self.measurement_noise_cov = np.array([[measurement_error[0]**2, 0],
                                               [0, measurement_error[1]**2]], dtype=float)

    def predict(self):
        # Predicción del siguiente estado
        self.state = np.dot(self.transition_matrix, self.state)
        # Predicción de la covarianza del siguiente estado
        self.covariance = np.dot(np.dot(self.transition_matrix, self.covariance),
                                 self.transition_matrix.T) + self.process_noise_cov

    def update(self, measurement):
        # Cálculo del error de la medición
        error = measurement - np.dot(self.measurement_matrix, self.state)
        # Cálculo de la matriz de ganancia de Kalman
        kalman_gain = np.dot(np.dot(self.covariance, self.measurement_matrix.T),
                             np.linalg.inv(np.dot(np.dot(self.measurement_matrix, self.covariance),
                                                  self.measurement_matrix.T) + self.measurement_noise_cov))
        # Actualización del estado y covarianza
        self.state = self.state + np.dot(kalman_gain, error)
        self.covariance = np.dot((np.eye(4) - np.dot(kalman_gain, self.measurement_matrix)),
                                 self.covariance)

        # Limitar la velocidad máxima
        self.state[1] = np.clip(self.state[1], -self.max_velocity, self.max_velocity)
        self.state[3] = np.clip(self.state[3], -self.max_velocity, self.max_velocity)

        return self.state[0], self.state[2], self.covariance  # Devolver las coordenadas filtradas
