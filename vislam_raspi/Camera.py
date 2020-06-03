import numpy as np

class Camera:

    def __init__(self, W, H, f, K):
        
        #Width and Height of frames
        self.width = W
        self.height = H

        self.focal_length = f

        #Camera intrinsic calibration matrix
        self.camera_intrinsic_matrix = np.matrix(K)
        self.inverse_camera_intrinsic_matrix = np.linalg.inv(self.camera_intrinsic_matrix)

        

