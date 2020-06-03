from helpers import poseRt, hamming_distance, add_ones
from constants import CULLING_ERR_THRES
from frame import Frame
import time
import numpy as np
import g2o
import json


# A Keypoint 
class Keypoint(object):
    def __init__(self, mapp, loc, color, tid=None):
        self.pt = np.array(loc)
        self.frames = []
        self.idxs = []
        self.color = np.copy(color)
        self.id = tid if tid is not None else mapp.add_point(self)

    def homogeneous(self):
        return add_ones(self.pt)

    def orb(self):
        return [f.des[idx] for f,idx in zip(self.frames, self.idxs)]

    def orb_distance(self, des):
        return min([hamming_distance(o, des) for o in self.orb()])
  
    def delete(self):
        for f,idx in zip(self.frames, self.idxs):
            f.pts[idx] = None
        del self

    def add_observation(self, frame, idx):
        assert frame.pts[idx] is None
        assert frame not in self.frames
        frame.pts[idx] = self
        self.frames.append(frame)
        self.idxs.append(idx)