import numpy as np

class KalmanFilter:
    def __init__(self, dt):
        self.A = np.array([[1, 0, dt,  0],
                           [0, 1,  0, dt],
                           [0, 0,  1,  0],
                           [0, 0,  0,  1]
                           ])
        self.C = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]
                           ])
        self.Q = np.diag([0.8, 0.8, 0.5, 0.5])
        self.R = 8 * np.eye(2)
    
    def kalman_filter(self, x_hat, P_hat, y_meas):
        x_check = self.A @ x_hat
        P_check = self.A @ P_hat @ self.A.T + self.Q

        K = (P_check @ self.C.T)  @ np.linalg.inv(self.C @ P_check @ self.C.T + self.R)

        P_hat = (np.eye(4) - K @ self.C) @ P_check
        x_hat = x_check + K @ (y_meas - self.C @ x_hat)
        
        return x_hat, P_hat

class LowPassFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
    
    def low_pass_filter(self, prev, curr):
        x = self.alpha*curr[0] + (1-self.alpha)*prev[0]
        y = self.alpha*curr[1] + (1-self.alpha)*prev[1]

        return x, y
