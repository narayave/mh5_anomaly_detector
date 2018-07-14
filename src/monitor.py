#!/usr/bin/env python

import sys
import rospy

from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from sklearn.externals import joblib
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import subprocess
import os

SCALER = None
REDUCE = None
MODEL = None
THRESHOLD = None

PREV = None
GOOD = 0
BAD = 0

POINTS = []
PREDS = []
POSITIONS_LIST = []


def load_modules(load_file, th):
	'''
		This function NEEDS to get called first, so that global variables can
			be set.
		Once set, they can be used to classify incoming data.
		Sets global variables, so they can be used later.
	'''

	global SCALER
	global REDUCE
	global MODEL
	global THRESHOLD

	THRESHOLD = th


	SCALER = joblib.load("./data/scale.pkl")
	REDUCE = joblib.load("./data/reduce.pkl")

	load_file = './data/'+load_file
	MODEL = joblib.load(load_file)

	# print 'Unsupervised models - ', rospy.get_param('unsupervised_model')
	# print 'Supervised models - ', rospy.get_param('supervised_model')

	# if load_file in rospy.get_param('unsupervised_model'):
	# 	MODEL_TYPE = 'unsupervised_model'
	# 	print 'Loaded an unsupervised model'

	# elif load_file in rospy.get_param('supervised_model'):
	# 	MODEL_TYPE = 'supervised_model'
	# 	print 'Loaded a supervised model'
	# else:
	# 	rospy.logwarn("WARNING: Model might not exist. Going to exit.")
	# 	rospy.signal_shutdown("Model type not known.")

	print 'Modules loaded. Over.'



def command(val):
	'''
		Callback function for when specified message is published.
		Adds data to global list to be processed later.
	'''
	global PREV
	poses = list(val.position)

	if PREV != poses:
		POSITIONS_LIST.append(poses)
		PREV = poses


def check():

	global SCALER
	global REDUCE
	global MODEL
	global THRESHOLD

	global GOOD
	global BAD

	global POINTS
	global PREDS

	while POSITIONS_LIST:


		data = np.array([POSITIONS_LIST[0]])
		POSITIONS_LIST.pop(0)

		scaled = SCALER.transform(data)
		# print 'scale done'

		pca_ed = REDUCE.transform(scaled)
		POINTS.append(pca_ed.tolist()[0])
		# print 'reduce done'

		prediction = MODEL.predict(pca_ed)
		PREDS.append(prediction[0])
		# print 'got prediction', prediction[0]

		if prediction == -1:
			GOOD += 1
			print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
		else:
			BAD += 1
			print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
			rospy.logwarn("WARNING: Robot state classified as an \
						anomaly.\n" + str(data))

		# FIXME: This logic is going to have to change.
		# 		Currently, this method does not take into account when an
		#		a single task is complete, but keeps a running tally.
		if (BAD/float(BAD+GOOD) > float(THRESHOLD)) and (BAD+GOOD > 60):
			rospy.logerr("ERR: Robot state classified as an \
							anomaly.\n" + str(data))


def graph_3d():

	global POINTS
	global PREDS
	global GOOD
	global BAD

	percentgood = GOOD/float(GOOD+BAD)
	percentbad = BAD/float(GOOD+BAD)
	print GOOD, BAD, percentgood, percentbad


	pts0 = [row[0] for row in POINTS]
	pts1 = [row[1] for row in POINTS]
	pts2 = [row[2] for row in POINTS]

	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.scatter(pts0, pts1, pts2, c=PREDS, cmap='bwr', s=10)

	plt.show()


if __name__ == '__main__':

	rospy.init_node('monitor', anonymous=True)

	load_modules(sys.argv[2], sys.argv[3])

	# Subscribe to the appropriate topic
	rospy.Subscriber(sys.argv[1],
				JointState,
				command)

	rospy.on_shutdown(graph_3d)

	while not rospy.is_shutdown():
		check()
