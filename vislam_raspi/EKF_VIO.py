from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter

from scipy.spatial.transform import Rotation

import numpy as np
import scipy
import Sympy


class EKF_VIO(ExtendedKalmanFilter):
    def __init__(self,state_vector_size_imu, no_of_state_landmarks):
        super().__init__(state_vector_size_imu + no_of_state_landmarks, no_of_state_landmarks)

        self.state_vector_size = state_vector_size_imu + no_of_state_landmarks
        self.state_vector_size_imu =state_vector_size_imu
        self.no_of_state_landmarks = no_of_state_landmarks
        
        self.process_noise_size = 12

        self.accecelation = np.array(np.zeros(3))
        self.angular_velocity = np.array(np.zeros(3))
        

        # self.state_landmarks_vector = np.array(np.zeros(6))
        # self.state_landmarks = np.array(np.zeros(self.no_of_state_landmarks))
        self.initState()


    def initState(self):

        #state variables
        self.orientation_quatenion = Rotation.from_quat([1, 0, 0, 0])
        self.position = np.array(np.zeros(3))
        self.velocity = np.array(np.zeros(3))
        self.bias_accel = np.array(np.zeros(3))
        self.bias_gyro = np.array(np.zeros(3))


        self.state_landmarks = []

        #initialize other EKF parameter P(covariance matrix)
        self.initEkfParameters()
    

    def initEkfParameters(self):
        #error state variables
        self.error_orientation_quatenion = Rotation.from_quat([1, 0, 0, 0])
        self.error_position = np.array(np.zeros(3))
        self.error_velocity = np.array(np.zeros(3))
        self.error_bias_accel = np.array(np.zeros(3))
        self.error_bias_gyro = np.array(np.zeros(3))

        #TODO
        self.rotation_world_imu = np.array(np.zeros(3))
        self.gravity_vector = np.array(np.zeros(3))

        #TODO 
        #diagonal covariance matrix of noise of accel, gyro, bias_accel, bias gyro
        self.Q = np.eye(self.process_noise_size )

        #idk what is this
        self.G = np.zeros((self.state_vector_size_imu,self.process_noise_size))

        #differentiation of Fundamental Matrix
        self.F_phi = np.zeros((self.state_vector_size_imu,self.state_vector_size_imu))

        #state covariance matrix
        self.P = np.zeros((self.state_vector_size,self.state_vector_size))


    @overrides(ExtendedKalmanFilter)
    def predict_x(self,accecelation, angular_velocity,delta_time):
        self.statePropagation(accecelation, angular_velocity,delta_time)
        
 
    def statePropagation(self,accecelation, angular_velocity,delta_time):
        
        #quaternion propatation
        delta_rotate = np.subtract(accecelation,self.bias_gyro)*delta_time
        self.orientation_quatenion *= Rotation.from_euler('XYZ',delta_rotate,degrees=True) 

        #unclear
        zeta = np.add(self.gravity_vector,np.multiply(self.rotation_world_imu,np.subtract(accecelation,self.bias_accel)))
        
        #position propatation
        self.position = np.add(np.add(self.position,self.velocity*delta_time),(0.5*(delta_time**2))*zeta)

        #velocity propatation
        self.velocity = np.add(self.velocity,zeta*delta_time)

        #extract covariance submatrix
        P_imu = self.P[0 : self.state_vector_size_imu,0 : self.state_vector_size_imu]
        P_imu_landmark = self.P[0  : self.state_vector_size_imu, self.state_vector_size_imu : ]
        P_landmark_imu = self.P[self.state_vector_size_imu : ,0 : self.state_vector_size_imu]

        #covariance submatrix propagation
        P_imu = np.add(np.dot(np.dot(self.F_phi,P_imu),self.F_phi.T),np.dot(np.dot(self.G,self.Q),self.G.T))
        P_imu_landmark = np.dot(self.F_phi,P_imu_landmark)
        P_landmark_imu = np.dot(P_landmark_imu,self.F_phi.T)


        #write submatrix back to covariance matrix
        self.P[0 : self.state_vector_size_imu,0 : self.state_vector_size_imu] = P_imu
        self.P[0  : self.state_vector_size_imu, self.state_vector_size_imu : ] = P_imu_landmark
        self.P[self.state_vector_size_imu : ,0 : self.state_vector_size_imu] = P_landmark_imu
        


    def filterUpdate(self):
        pass         


