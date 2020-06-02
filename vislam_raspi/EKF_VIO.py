from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import scipy
import Sympy


class EKF_VIO(ExtendedKalmanFilter):
    def __init__(self,state_vector_size, no_of_state_landmarks):
        super().__init__(state_vector_size, no_of_state_landmarks)

        self.state_vector_size = state_vector_size
        self.no_of_state_landmarks = no_of_landmarks
        

        self.accecelation = np.array(np.zeros(3))
        self.angular_velocity = np.array(np.zeros(3))
        

        # self.state_landmarks_vector = np.array(np.zeros(6))
        self.state_landmarks = np.array(np.zeros(self.no_of_state_landmarks))

        # self.state_vector = 
    def initState(self):
        self.orientation_quatenion = np.array(np.zeros(4))
        self.position = np.array(np.zeros(3))
        self.velocity = np.array(np.zeros(3))
        self.bias_accel = np.array(np.zeros(3))
        self.bias_gyro = np.array(np.zeros(3))

    def init_imu_ekf(self):
         



    @overrides(ExtendedKalmanFilter)
    def predict_x()



    
