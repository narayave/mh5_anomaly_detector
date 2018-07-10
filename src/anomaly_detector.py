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


class AnomalyDetector(object):

	def __init__(self,
			unsup_models = [('OC SVM', svm.OneClassSVM(nu=0.001, kernel="rbf", gamma=21))],
			sup_models = [('RBF SVM', svm.SVC(kernel='rbf', gamma=1000, C=100000))]):
		self.__unsup_clfs = unsup_models
		self.__sup_clfs = sup_models
		self.outputMat__ = OutputMatrix()
		rospy.set_param('unsupervised_model', str(['None']))
		rospy.set_param('supervised_model', str(['None']))


	# TODO: This was to help with graphing
	def get_OCSVM(self):
		return self.__unsup_clfs[0][1]

	def get_RBFSVM(self):
		return self.__sup_clfs[0][1]


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


	def classify(self, data_x):
		'''
			Pass in a set of data and get predictions for 2 models
		'''
		print 'data_X', data_x.shape
		ocsvm_predictions = self.ocsvm_predict(data_x)
		rbfsvm_predictions = self.rbfsvm_predict(data_x)

		return ocsvm_predictions, rbfsvm_predictions


	def classify_supervised(self, data):
		'''
			Get predictions for unsupervised models.
			Calls 'private' method with model for predictions.
				Returned predictions are compiled together
		'''
		all_preds = []
		for i in xrange(0, len(self.__sup_clfs)):
			model = self.__sup_clfs[i]
			preds = self.__classify('supervised', model, data)
			all_preds.append(preds)

		all_preds = np.array(all_preds)
		print 'Supervised preds: ', all_preds.shape
		return all_preds


	def classify_unsupervised(self, data):
		'''
			Get predictions for unsupervised models.
			Calls 'private' method with model for predictions.
				Returned predictions are compiled together
		'''
		all_preds = []
		for i in xrange(0, len(self.__unsup_clfs)):
			model = self.__unsup_clfs[i]
			preds = self.__classify('unsupervised', model, data)
			all_preds.append(preds)

		all_preds = np.array(all_preds)
		print 'Unupervised preds: ', all_preds.shape
		return all_preds


	def __classify(self, mod_type, model, data):
		'''
			Given a model, prediction is made.

			BUG: Assuming model is used as a binary classifier,
					predictions are turned to 0/1 vs 1/-1
		'''
		predictions = model[1].predict(data)

		if mod_type == 'unsupervised':
			for i in xrange(len(predictions)):
				# print predictions[i]
				if predictions[i] == 1:
					predictions[i] = 0
				else:
					predictions[i] = 1

		if mod_type == 'supervised':
			for i in xrange(len(predictions)):
				# print predictions[i]
				if predictions[i] == 1:
					predictions[i] = 1
				else:
					predictions[i] = 0


		errs = predictions[predictions == 1].size
		print model[0] + ' Errors: ', 100*float(errs)/data.shape[0]
		return predictions


	def ocsvm_predict(self, data):
		''' Spit out predictions in '''
		# print data.shape
		predictions = self.__unsup_clfs[0][1].predict(data)

		# NOTE: This is to make sure predictions are all 0 and 1s.
		#		0 is benign/expected. 1 is an anomaly.
		for i in xrange(len(predictions)):
			if predictions[i] == 1:
				predictions[i] = 1
			else:
				predictions[i] = 0

		oc_errors = predictions[predictions == 1].size
		print 'OCSVM errors: ', 100*float(oc_errors)/data.shape[0]
		return predictions


	def rbfsvm_predict(self, data):
		predictions = self.__sup_clfs[0][1].predict(data)

		for i in xrange(len(predictions)):
			if predictions[i] == 1:
				predictions[i] = 1
			else:
				predictions[i] = 0

		rbf_errors = predictions[predictions == 1].size
		print 'RBFSVM errors: ', 100*float(rbf_errors)/data.shape[0]
		return predictions


	def call_output(self, true_y, pred_y):
		'''
			Predictions for multiple classifiers are being
				passed, so need to iterate through them.
		'''
		for item in pred_y:
			self.outputMat__.output_matrix(true_y, item)
