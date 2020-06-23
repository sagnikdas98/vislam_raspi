import os

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform
from skimage.transform import EssentialMatrixTransform


from constants import RANSAC_RESIDUAL_THRES, RANSAC_MAX_TRIALS
np.set_printoptions(suppress=True)
from Utils import add_ones, poseRt, fundamentalToRt, normalize, EssentialMatrixTransform, myjet



class Frame(object):   

	#static data member TODO: MAP Declaration
	MAP = None
	CAMERA_INTRINSIC_MATRIX = None
	ORB = cv2.ORB_create()
	BF_MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING)
	MIN_ORB_DISTANCE = 32
	
	#KEYFRAME LIST TODO: cotriant the list of stored frames
	
	KEYFRAMES_LIST = []


	#RANSAC parameters
	RANSAC_MIN_SAMPLES = 8
	RANSAC_RESIDUAL_THRES = 0.02
	RANSAC_MAX_TRIALS = 100

	@staticmethod
	def fundamentalToRt(F):
		W = np.mat([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
		U,d,Vt = np.linalg.svd(F)
		if np.linalg.det(U) < 0:
			U *= -1.0
		if np.linalg.det(Vt) < 0:
			Vt *= -1.0
		R = np.dot(np.dot(U, W), Vt)
		if np.sum(R.diagonal()) < 0:
			R = np.dot(np.dot(U, W.T), Vt)
		t = U[:, 2]

		# TODO: Resolve ambiguities in better ways. This is wrong.
		if t[2] < 0:
			t *= -1
		
		# TODO: UGLY!
		if os.getenv("REVERSE") is not None:
			t *= -1
		return np.linalg.inv(poseRt(R, t))




	@staticmethod
	def matchFrames(src_frame, dest_frame):
		#TODO:Add exception ransac wont work if min matches less than 8; keyframe cond

		#src_frame: query, dest_frame:train
		matches = Frame.BF_MATCHER.knnMatch(src_frame.keypoint_descriptors, dest_frame.keypoint_descriptors, k=2)

		matches_queryidx_dictionary = {}
		matches_trainidx_dictionary = {}

		#store the index of matched points (src_frame_point, dest_frame_point)
		matched_point_pair = []


		#Lowe's ratio test
		for match_first, match_second in matches:
			#first match is significantly closer than the second match
			if match_first.distance < 0.75*match_second.distance:

				#Within minimum ORB distance
				if match_first.distance < Frame.MIN_ORB_DISTANCE:
					
					#using dictionary to reduce search time complexity of point pairs
					if (str(match_first.trainIdx) not in matches_trainidx_dictionary) :
						matches_queryidx_dictionary[str(match_first.queryIdx)] = {'trainIdx' : match_first.trainIdx, 'distance' : match_first.distance}
						matches_trainidx_dictionary[str(match_first.trainIdx)] = {'queryIdx' : match_first.queryIdx, 'distance' : match_first.distance}

					else:
						if match_first.distance < matches_trainidx_dictionary[str(match_first.trainIdx)]['distance']:
							del matches_queryidx_dictionary[matches_trainidx_dictionary[str(match_first.trainIdx)]['queryIdx']]
							matches_queryidx_dictionary[str(match_first.queryIdx)] = {'trainIdx' : match_first.trainIdx, 'distance' : match_first.distance}
							matches_trainidx_dictionary[str(match_first.trainIdx)] = {'queryIdx' : match_first.queryIdx, 'distance' : match_first.distance}
		
		for i in matches_queryidx_dictionary.keys():
			matched_point_pair.append([src_frame.keypoints[int(i)], src_frame.keypoints[matches_queryidx_dictionary[i]['trainIdx']]])
		

		#convert matched points to numpy array
		matched_point_pair = np.array(matched_point_pair)

		#pass src dest 2 to rasac 
		src_points = matched_point_pair[:, 0] #src_frame
		dest_points = matched_point_pair[:, 1] #dest_frame

		return  src_points, dest_points




	@staticmethod
	def extractFeatures(image):

		#check if image is 3 channel (RGB)
		if image.shape == 3:
			#convert to grey scale as cv2.goodFeaturesToTrack() takes single channel image
			image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			points = cv2.goodFeaturesToTrack(image_gray, 100, qualityLevel=0.1, minDistance=7)
		else:
			points = cv2.goodFeaturesToTrack(image, 100, qualityLevel=0.1, minDistance=7)

		#feature description (fingerprint)
		keypoints = [cv2.KeyPoint(x=p[0][0], y=p[0][1], _size=20) for p in points]
		keypoints, keypoint_descriptors = Frame.ORB.compute(image, keypoints)
		#convert back to list of tuples
		keypoints = [(int(kp.pt[0]), int(kp.pt[1])) for kp in keypoints]

		return keypoints, keypoint_descriptors

	
	def __init__(self, image, time_stamp=None,is_keyframe = False):

		self.time_stamp = time_stamp
		self.pose_quaternion = Rotation.from_quat([1, 0, 0, 0])
		self.is_keyframe = is_keyframe

		#list of keypoints and descriptor as a tuple ((x,y),[descriptor])
		self.keypoints, self.keypoint_descriptors = Frame.extractFeatures(image)
		
		#TODO
		self.points = [None]*len(self.keypoints)
		#TODO: Should be independent of MAP; Implement driver main function
		self.id = Frame.MAP.add_frame(self)


	def computeTransform(self):
		src_frame = Frame.KEYFRAMES_LIST[-1]
		dest_frame = self

		#get matched points w.r.t previous frame
		src_points, dest_points = Frame.matchFrames(src_frame,dest_frame)

		#fit matrix
		model_matrix, bool_inliers_mask = ransac((src_points, dest_points),
														FundamentalMatrixTransform,
														min_samples=Frame.RANSAC_MIN_SAMPLES,
														residual_threshold=Frame.RANSAC_RESIDUAL_THRES,
														max_trials=Frame.RANSAC_MAX_TRIALS)

		#getpose
		delta_rotate = fundamentalToRt(model_matrix.matrix)
		delta_rotate = Rotation.from_euler('XYZ',delta_rotate,degrees=True) 
		self.pose_quaternion = delta_rotate*Frame.KEYFRAMES_LIST[-1].pose_quaternion

		return src_points[bool_inliers_mask], dest_points[bool_inliers_mask], 



	def checkForKeyframe(self):
		#TODO
		return self.is_keyframe

	def annotate(self, img):
		# paint annotations on the image
		for i1 in range(len(self.kpus)):
			u1, v1 = int(round(self.kpus[i1][0])), int(round(self.kpus[i1][1]))
			if self.pts[i1] is not None:
				if len(self.pts[i1].frames) >= 5:
					cv2.circle(img, (u1, v1), color=(0,255,0), radius=3)
				else:
					cv2.circle(img, (u1, v1), color=(0,128,0), radius=3)
				# draw the trail
				pts = []
				lfid = None
				for f, idx in zip(self.pts[i1].frames[-9:][::-1], self.pts[i1].idxs[-9:][::-1]):
					if lfid is not None and lfid-1 != f.id:
						break
					pts.append(tuple(map(lambda x: int(round(x)), f.kpus[idx])))
					lfid = f.id
				if len(pts) >= 2:
					cv2.polylines(img, np.array([pts], dtype=np.int32), False, myjet[len(pts)]*255, thickness=1, lineType=16)
			else:
				cv2.circle(img, (u1, v1), color=(0,0,0), radius=3)
		return img


	# inverse of intrinsics matrix
	@property
	def Kinv(self):
		if not hasattr(self, '_Kinv'):
			self._Kinv = np.linalg.inv(self.K)
		return self._Kinv

	# normalized keypoints
	@property
	def kps(self):
		if not hasattr(self, '_kps'):
			self._kps = normalize(self.Kinv, self.kpus)
		return self._kps

	# KD tree of unnormalized keypoints
	@property
	def kd(self):
		if not hasattr(self, '_kd'):
			self._kd = cKDTree(self.kpus)
		return self._kd

