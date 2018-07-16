#!/usr/bin/env python

'''
Ensemble Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Example of training script for ROS module
Date:
	14 May, 2018

'''


import sys
from interface import Interface
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from sklearn import svm
import yaml
import rospy
from sklearn.decomposition import PCA
import os

def graph_3d(test, y_pred_test):

	print test.shape
	print y_pred_test.shape

	te0, te1, te2 = test[:, 0], test[:, 1], test[:, 2]
	y_pred_test = y_pred_test[0]

	fig = plt.figure()
	ax = fig.gca(projection='3d')

	ax.scatter(te0, te1, te2, c=y_pred_test, \
	cmap='bwr', \
	s=10)

	plt.show()


def reduce_graph_3d(data, preds):

	pca = PCA(n_components=3)

	pca.fit(data)
	data = pca.transform(data)

	graph_3d(data, preds)



if __name__ == "__main__":

	unsupervised_models = []
	supervised_models = []


	print os.getcwd()
	print sys.argv

	input_file = sys.argv[1] #"learning_script.yaml"
	interface = Interface(input_file)
	# interface.print_all()


	with open(input_file, 'r') as fil:
		local = yaml.load(fil)

		fit_file = local["Files"]["fit_file"]
		unsupervised_train_file = local["Files"]["unsupervised_train"]
		supervised_train_file = local["Files"]["supervised_train"]
		testing_file = local["Files"]["testing"]

	clf_ocsvm17 = svm.OneClassSVM(nu=0.05, kernel="rbf", gamma=1000)
	unsupervised_models.append(('ocsvm17', clf_ocsvm17))
	clf_ocsvm18 = svm.OneClassSVM(nu=0.06, kernel="rbf", gamma=1100)
	unsupervised_models.append(('ocsvm18', clf_ocsvm18))


	clf_rbfsvm1 = svm.SVC(kernel='rbf', gamma=150, C=100000)
	supervised_models.append(('clf_rbfsvm1', clf_rbfsvm1))
	# clf_rbfsvmbest_3 = svm.SVC(kernel='rbf', gamma=100, C=100000)
	# supervised_models.append(('rbfsvmbest_3', clf_rbfsvmbest_3))

	interface.genmodel_train(unsupervised_models, supervised_models)

	# TODO: Allow for getting predictions for a particular model
	#		This will ideally be ignored, because learned will be
	#			in the monitoring phase.
	# new_item = "data/rbfsvm_train_20.csv"
	u_preds, s_preds = interface.get_testing_predictions()

	# print testing_file
	# print testing_file[0]["name"]
	x, y = interface.retrieve_data(testing_file[0]["name"])

	print 'x shape: ', x.shape
	print 'y shape: ', np.array(y).shape

	# print rospy.get_param('unsupervised_model')
	# print rospy.get_param('supervised_model')

	# graph_3d(x, s_preds[0])

	reduce_graph_3d(x, u_preds[0])
