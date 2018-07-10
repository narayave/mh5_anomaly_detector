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
MODEL_TYPE = None
THRESHOLD = None

PREV = None
GOOD = 0
BAD = 0

POINTS = []
PREDS = []
POSITIONS_LIST = []

'''
	This function NEEDS to get called first, so that global variables can
		be set.
	Once set, they can be used to classify incoming data.
	Sets global variables, so they can be used later.
'''
def load_modules(load_file, th):

	global SCALER
	global REDUCE
	global MODEL
	global MODEL_TYPE
	global THRESHOLD

	THRESHOLD = th


	SCALER = joblib.load("./data/scale.pkl")
	REDUCE = joblib.load("./data/reduce.pkl")

	load_file = './data/'+load_file
	MODEL = joblib.load(load_file)

	print 'Unsupervised models - ', rospy.get_param('unsupervised_model')
	print 'Supervised models - ', rospy.get_param('supervised_model')

	if load_file in rospy.get_param('unsupervised_model'):
		MODEL_TYPE = 'unsupervised_model'
		print 'Loaded an unsupervised model'

	elif load_file in rospy.get_param('supervised_model'):
		MODEL_TYPE = 'supervised_model'
		print 'Loaded a supervised model'
	else:
		rospy.logwarn("WARNING: Model might not exist. Going to exit.")
		rospy.signal_shutdown("Model type not known.")

	print 'Modules loaded. Over.'


'''
	Callback function for when specified message is published.
	Adds data to global list to be processed later.
'''
def command(val):

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
	global MODEL_TYPE


	while POSITIONS_LIST:

		# print len(POSITIONS_LIST), 'POSITIONS_LIST'

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


		# FIXME: Branches are generally bad, not sure of another way to program
		#			Prediction values are different for unsuper vs supervised

		if MODEL_TYPE == 'unsupervised_model':
			if prediction == 1:
				GOOD += 1
				print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
				print GOOD, 'GOOD', BAD, 'BAD'
			elif prediction == -1:
				BAD += 1
				print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
				rospy.logwarn("WARNING: Robot state classified as an \
						anomaly.\n" + str(data))


		elif MODEL_TYPE == 'supervised_model':
			if prediction == 0:
				GOOD += 1
				print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
			elif prediction == 1:
				BAD += 1
				print 'Good: ', GOOD/float(GOOD+BAD), 'Bad: ', BAD/float(GOOD+BAD), 'Count: ', GOOD+BAD
				rospy.logwarn("WARNING: Robot state classified as an \
						anomaly.\n" + str(data))


		if (BAD/float(BAD+GOOD) > float(THRESHOLD)) and (BAD+GOOD > 60):
			rospy.logerr("ERR: Robot state classified as an \
						anomaly.\n" + str(data))

		# 	print 'Threshold has been passed. Anomalous run. Ending.'

		# 	nodes = os.popen("rosnode list").readlines()
		# 	for i in range(len(nodes)):
		# 		nodes[i] = nodes[i].replace("\n","")

		# 	for node in nodes:
		# 		if node != 'monitor':
		# 			os.system("rosnode kill " + node)


def make_meshgrid(x, y, h=.02):
	"""Create a mesh of points to plot in

	Parameters
	----------
	x: data to base x-axis meshgrid on
	y: data to base y-axis meshgrid on
	h: stepsize for meshgrid, optional

	Returns
	-------
	xx, yy : ndarray
	"""
	x_min, x_max = x.min() - 0.25, x.max() + 0.25
	y_min, y_max = y.min() - 0.25, y.max() + 0.25
	xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
						 np.arange(y_min, y_max, h))
	return xx, yy


def graph():

	global MODEL

	global POINTS
	global PREDS
	global GOOD
	global BAD

	percentgood = GOOD/float(GOOD+BAD)
	print GOOD, BAD, percentgood, BAD/float(GOOD+BAD)

	POINTS = np.array(POINTS)
	PREDS = np.array(PREDS).reshape((-1,))
	print POINTS.shape, PREDS.shape

	pts0, pts1 = POINTS[:, 0], POINTS[:, 1]
	xx, yy = make_meshgrid(pts0, pts1, h=0.01)


	# plot the line, the points, and the nearest vectors to the plane
	Z = MODEL.decision_function(np.c_[xx.ravel(), yy.ravel()])
	Z = Z.reshape(xx.shape)

	fig = plt.figure()

	plt.contourf(xx, yy, Z, levels=np.linspace(Z.min(), 1, 7), cmap=plt.cm.PuBu)
	a = plt.contour(xx, yy, Z, levels=[0], linewidths=2, colors='darkred')
	plt.contourf(xx, yy, Z, levels=[0, Z.max()], colors='palevioletred')

	plt.scatter(pts0, pts1, c=PREDS, cmap='spring', s=10)

	plt.show()


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
				JointState,			# TODO: This needs to get specified too?
				command)

	rospy.on_shutdown(graph_3d)

	while not rospy.is_shutdown():
		check()
