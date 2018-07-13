'''
ROS Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Anomaly Detector class
Date:
	4 May, 2018

'''

import os
from sklearn import svm
import numpy as np
from output_matrix import OutputMatrix
from sklearn.externals import joblib
import rospy
from pprint import pprint


class AnomalyDetector(object):

	def __init__(self, unsup_models, sup_models):
			# unsup_models = [('OC SVM', svm.OneClassSVM(nu=0.001, kernel="rbf", gamma=21))],
			# sup_models = [('RBF SVM', svm.SVC(kernel='rbf', gamma=1000, C=100000))]):
		self.__unsup_clfs = unsup_models
		self.__sup_clfs = sup_models
		self.__outputMat = OutputMatrix()
		rospy.set_param('unsupervised_model', str(['None']))
		rospy.set_param('supervised_model', str(['None']))


	def fit_supervised_models(self, sup_train_x, sup_train_y):
		'''
			Unsupervised data is mixed. Labels (y) are passed in.
		'''
		self.__fit_supervised(sup_train_x, sup_train_y)


	def fit_unsupervised_models(self, data_x):
		'''
			Supervised data is meant to all be safe. No labels.
		'''
		self.__fit_unsupervised(data_x)


	def __fit_unsupervised(self, data):
		'''
			'Private' fit unsupervised models method
			Iterate through all models
		'''
		unsuper_clfs = []
		for i in xrange(0, len(self.__unsup_clfs)):
			current_model = self.__unsup_clfs[i]

			# NOTE: If model exists, load. or fit and save
			saveloc = './data/'+current_model[0]+'.pkl'
			if os.path.exists(saveloc):
				current_model = (current_model[0], joblib.load(saveloc))
			else:
				current_model[1].fit(data)
				joblib.dump(current_model[1], saveloc)

			# Adding fitted model back to class member
			self.__unsup_clfs[i] = current_model
			unsuper_clfs.append(saveloc)
		rospy.set_param('unsupervised_model', str(unsuper_clfs))


	def __fit_supervised(self, data_x, data_y):
		'''
			'Private' fit supervised models method
			Iterate through all models
		'''
		super_clfs = []
		for i in xrange(0, len(self.__sup_clfs)):
			current_model = self.__sup_clfs[i]

			# NOTE: If model exists, load. or fit and save
			saveloc = './data/'+current_model[0]+'.pkl'
			if os.path.exists(saveloc):
				current_model = (current_model[0], joblib.load(saveloc))
			else:
				current_model[1].fit(data_x, data_y.ravel())
				joblib.dump(current_model[1], saveloc)

			# Adding fitted model back to class member
			self.__sup_clfs[i] = current_model
			super_clfs.append(saveloc)
		rospy.set_param('supervised_model', str(super_clfs))


	def classify_supervised(self, dataset_name, in_data, out_label):
		'''
			Get predictions for unsupervised models.
			Calls 'private' method with model for predictions.
				Returned predictions are compiled together
		'''

		all_preds = []
		for i in xrange(0, len(self.__sup_clfs)):
			model = self.__sup_clfs[i]
			preds = self.__classify(model, in_data)
			all_preds.append(preds)

		all_preds = np.array(all_preds)

		print 'Supervised results for', dataset_name
		print 'Order of results: ', [item[0] for item in self.__sup_clfs]
		self.call_output(out_label, all_preds)
		return all_preds


	def classify_unsupervised(self, dataset_name, in_data, out_label):
		'''
			Get predictions for unsupervised models.
			Calls 'private' method with model for predictions.
				Returned predictions are compiled together
		'''

		all_preds = []
		for i in xrange(0, len(self.__unsup_clfs)):
			model = self.__unsup_clfs[i]
			preds = self.__classify(model, in_data)
			all_preds.append(preds)

		all_preds = np.array(all_preds)

		print 'Unsupervised results for', dataset_name
		print 'Order of results: ', [item[0] for item in self.__unsup_clfs]
		self.call_output(out_label, all_preds)
		return all_preds


	def __classify(self, model, data):
		'''
			Given a model, prediction is made.

			BUG: Assuming model is used as a binary classifier,
					predictions are turned to 0/1 vs 1/-1
		'''
		predictions = model[1].predict(data)


		errs = predictions[predictions == 1].size
		# print model[0] + ' Errors: ', 100*float(errs)/data.shape[0]
		return predictions



	def call_output(self, true_y, pred_y):
		'''
			Predictions for multiple classifiers are being
				passed, so need to iterate through them.
		'''
		for item in pred_y:
			self.__outputMat.output_matrix(true_y, item)
