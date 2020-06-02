import filterpy
import numpy as np
import scipy


class State:
    def __init__(self):


        self.no_of_state_landmarks = 30 #variable
        self.full_state_vector_size = 16 + self.no_of_state_landmarks*6

        self.position = np.array(np.zeros(3))
        self.velocity = np.array(np.zeros(3))
        self.accecelation = np.array(np.zeros(3))
        self.orientation_quatenion = np.array(np.zeros(4))
        self.angular_velocity = np.array(np.zeros(3))
        
        self.no_of_state_landmarks = 30 #variable
        self.state_landmarks_vector = np.array(np.zeros(6))
        self.state_landmarks = np.array(np.zeros(self.no_of_state_landmarks))
        self.state_vector = 

    def init_imu_ekf(self):
        self.imu_ekf_obj = filterpy.kalman.ExtendedKalmanFilter(self.full_state_vector_size,self.no_of_state_landmarks)



    def imu_ekf(self):


    
