'''
ROS Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Model Interface class
Date:
	13 May, 2018


NOTE:
	- This interface is built to help the user.
	- This is meant to automate things. Since training files have
		already been passed, the data can be made ready for the user
		to use.
'''

from pprint import pprint
import yaml
from sklearn import svm

from data_process import DataProcess
from anomaly_detector import AnomalyDetector
from data_persist import DataPersist

class Interface(object):

	def __init__(self, input_file):

		with open(input_file, 'r') as fil:
			local = yaml.load(fil)

			self.__fit_file = local["Files"]["fit_file"]
			self.__unsupervised_train_file = local["Files"]["unsupervised_train"]
			self.__supervised_train_file = local["Files"]["supervised_train"]
			self.__testing = local["Files"]["testing"]
			self.__processor = DataProcess(local["Operations"]["reduction"])

		self.__anomaly_classifier = None
		self.__data_persist = DataPersist()

		self.__ready_preprocessor()
		self.__preprocess_data_training()


	def print_all(self):
		'''
			This was written primarily as a sanity check.
			That's the only purpose it serves.
		'''

		print 'fit\t', self.__fit_file, type(self.__fit_file)
		print 'unsuper\t', self.__unsupervised_train_file, type(self.__unsupervised_train_file)
		print 'super\t', self.__supervised_train_file, type(self.__supervised_train_file)
		print 'test\t', self.__testing, type(self.__testing)
		pprint(self.__testing)
		print '\n'


	def __ready_preprocessor(self):
		'''
			NOTE: Specifically for fitting models
				- Split
				- Scale
				- Reduce
				- BUG: Should this data be saved for future?

			NOTE: Functions below are specifically calling fit.
		'''

		split_x, split_y = self.__processor.split(self.__fit_file["name"], \
					self.__fit_file["input_col"], \
					self.__fit_file["output_col"])

		scaled_x = self.__processor.scaler_fit(split_x)
		reduced_x = self.__processor.reduction_fit(scaled_x)
		self.__data_persist.dump_data(self.__fit_file["name"], \
							reduced_x, split_y)


	def __preprocess_data_training(self):
		'''
			Private function, user does not need to know this.
			Use this function to get training data ready.
			Checks to make sure a training file has been passed.
				If filename is Null/None, it is skipped.

		'''

		if self.__unsupervised_train_file["name"]:
			self.__preprocess_data_helper(self.__unsupervised_train_file)

		if self.__supervised_train_file["name"]:
			self.__preprocess_data_helper(self.__supervised_train_file)


	def __preprocess_data_helper(self, item):
		'''
			NOTE: Can only be called if __ready_processor has been called.
				It should be, when the instance is created.
		'''

		split_x, split_y = self.__processor.split(item["name"], \
					item["input_col"], \
					item["output_col"])
		scaled_x = self.__processor.scaler_transform(split_x)
		reduced_x = self.__processor.reduction_transform(scaled_x)
		self.__data_persist.dump_data(item["name"], reduced_x, split_y)


	def genmodel_train(self, unsup_models, sup_models):
		'''
			Creates an instance of anomaly classifier.
			Training data files already exist, so all models can and are
				trained.
			Models ready for predictions after this stage.

			NOTE: Training predictions are grabbed and outputs are trying
				to get printed.
				Once the process is set, the predictions can be ridden.
		'''

		self.__anomaly_classifier = AnomalyDetector(unsup_models, sup_models)


		# Only need to do this, if an unsupervised method is passed
		if self.__unsupervised_train_file["name"]:
			# NOTE: The following can be scratched once the process is set
			print '=====Unsupervised====='
			unsup_x, unsup_y = self.__data_persist.retrieve_dumped_data(
									self.__unsupervised_train_file["name"])
			self.__anomaly_classifier.fit_unsupervised_models(unsup_x)
			preds = self.__anomaly_classifier.classify_unsupervised(unsup_x)
			self.__anomaly_classifier.call_output(unsup_y, preds)


		# Only need to do this, if an supervised method is passed
		if self.__supervised_train_file["name"]:
			print '=====Supervised====='
			sup_x, sup_y = self.__data_persist.retrieve_dumped_data(
								self.__supervised_train_file["name"])
			self.__anomaly_classifier.fit_supervised_models(sup_x, sup_y)
			preds = self.__anomaly_classifier.classify_supervised(sup_x)
			self.__anomaly_classifier.call_output(sup_y, preds)


	def get_testing_predictions(self):
		'''
			Get predictions for testing files that was passed in
		'''
		u_preds = []
		s_preds = []

		for item in self.__testing:
			print item["name"]
			self.__preprocess_data_helper(item)
			usup_preds, sup_preds = self.__testing_predictions_helper(item)

			u_preds.append(usup_preds)
			s_preds.append(sup_preds)

		return u_preds, s_preds


	def __testing_predictions_helper(self, item):

		unsuper_preds, super_preds = [], []

		x, y = self.__data_persist.retrieve_dumped_data(item["name"])

		if self.__unsupervised_train_file["name"]:
			unsuper_preds = self.__anomaly_classifier.classify_unsupervised(x)
			print 'Unsupervised', item["name"]
			self.__anomaly_classifier.call_output(y, unsuper_preds)

		if self.__supervised_train_file["name"]:
			super_preds = self.__anomaly_classifier.classify_supervised(x)
			print 'Supervised', item["name"]
			self.__anomaly_classifier.call_output(y, super_preds)

		print ''

		return unsuper_preds, super_preds


	def retrieve_data(self, loc):
		return self.__data_persist.retrieve_dumped_data(loc)
