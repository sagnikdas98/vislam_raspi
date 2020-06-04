import os

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from constants import RANSAC_RESIDUAL_THRES, RANSAC_MAX_TRIALS

np.set_printoptions(suppress=True)

from skimage.measure import ransac

from Utils import add_ones, poseRt, fundamentalToRt, normalize, EssentialMatrixTransform, myjet



class Frame(object):   

	#static data member TODO: MAP Declaration
	MAP = None
	CAMERA_INTRINSIC_MATRIX = None
	ORB = cv2.ORB_create()
	BF_MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING)
	MIN_ORB_DISTANCE = 32




	@staticmethod
	def matchFrames(frame1, frame2):

		matches = Frame.BF_MATCHER.knnMatch(frame1.keypoint_descriptors, frame2.keypoint_descriptors, k=2)

		matches_queryidx_dictionary = {}
		matches_trainidx_dictionary = {}

		#store the index of matched points (frame1_point, frame2_point)
		matched_point_pair = []


		#Lowe's ratio test
		for match_first, match_second in matches:
			#first match is significantly closer than the second match
			if match_first.distance < 0.75*match_second.distance:
				# frame1_point = frame1.keypoints[match_first.queryIdx]
				# frame2_point = frame2.keypoints[match_first.trainIdx]

				#Within minimum ORB distance
				if match_first.distance < Frame.MIN_ORB_DISTANCE:
					
					#using dictionary to reduce search time complexity
					if (str(match_first.trainIdx) not in matches_trainidx_dictionary) :
						matches_queryidx_dictionary[str(match_first.queryIdx)] = {'trainIdx' : match_first.trainIdx, 'distance' : match_first.distance}
						matches_trainidx_dictionary[str(match_first.trainIdx)] = {'queryIdx' : match_first.queryIdx, 'distance' : match_first.distance}

					else:
						if match_first.distance < matches_trainidx_dictionary[str(match_first.trainIdx)]['distance']:
							del matches_queryidx_dictionary[matches_trainidx_dictionary[str(match_first.trainIdx)]['queryIdx']]
							matches_queryidx_dictionary[str(match_first.queryIdx)] = {'trainIdx' : match_first.trainIdx, 'distance' : match_first.distance}
							matches_trainidx_dictionary[str(match_first.trainIdx)] = {'queryIdx' : match_first.queryIdx, 'distance' : match_first.distance}
		
		for i in matches_queryidx_dictionary.keys():
			matched_point_pair.append([int(i),matches_queryidx_dictionary[i]['trainIdx'] ])
		

		#convert matched points to numpy array
		matched_point_pair = np.array(matched_point_pair)


		#fit matrix
		model, inliers = ransac((ret[:, 0], ret[:, 1]),
														EssentialMatrixTransform,
														min_samples=8,
														residual_threshold=RANSAC_RESIDUAL_THRES,
														max_trials=RANSAC_MAX_TRIALS)
		print("Matches:  %d -> %d -> %d -> %d" % (len(f1.des), len(matches), len(inliers), sum(inliers)))
		return idx1[inliers], idx2[inliers], fundamentalToRt(model.params)

	@staticmethod
	def extractFeatures(image):

		#convert to grey scale as cv2.goodFeaturesToTrack() takes single channel image
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		points = cv2.goodFeaturesToTrack(image_gray, 100, qualityLevel=0.1, minDistance=7)

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
		self.id = Frame.MAP.add_frame(self)

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

