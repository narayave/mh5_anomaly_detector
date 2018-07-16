'''
ROS Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Data Process class
Date:
	4 May, 2018
'''

import os
from sklearn.preprocessing import MinMaxScaler
from sklearn.decomposition import PCA
from sklearn.utils import shuffle
import numpy as np
from sklearn.externals import joblib
import pandas as pd


class DataProcess(object):
	"""
		Specifically for processing data and getting it ready
			for anomaly detection.
	"""

	def __init__(self, comps):
		self.__scale = MinMaxScaler(feature_range=(0, 1))
		self.__reduction_size = comps

		if comps > 0:
			self.__reduction = PCA(n_components=comps)
		else:
			self.__reduction = None


	def split(self, in_file, x_cols, y_cols):
		''' Read csv, to dataframe, split to x and y. '''

		dataframe = pd.read_csv(in_file, engine='python')
		dataset = dataframe.values
		data = shuffle(dataset)

		# Gets rid of redundant entries, more accuracte
		data = np.unique(data, axis=0)

		dataframe_x = data[:, x_cols]
		dataframe_y = data[:, y_cols]

		return dataframe_x, dataframe_y


	def scaler_fit(self, data):
		''' Fit scaling object '''
		# NOTE: If model exists, load. or fit and save

		print os.getcwd()
		saveloc = '../data/scale.pkl'
		if os.path.exists(saveloc):
			self.__scale = joblib.load(saveloc)
			data = self.__scale.transform(data)
		else:
			data = self.__scale.fit_transform(data)
			joblib.dump(self.__scale, saveloc)

		return data


	def scaler_transform(self, data):
		''' Data supplied is scaled and returned. '''
		transformed_data = self.__scale.transform(data)
		return transformed_data


	def reduction_fit(self, data):
		''' Fit PCA decomposition. '''
		# NOTE: If model exists, load. or fit and save

		if self.__reduction:
			saveloc = '../data/reduce.pkl'
			if os.path.exists(saveloc):
				self.__reduction = joblib.load(saveloc)
				data = self.__reduction.transform(data)
			else:
				data = self.__reduction.fit(data)
				joblib.dump(self.__reduction, saveloc)

		return data


	def reduction_transform(self, data):
		''' Reduce dimensionality of given data. '''

		if self.__reduction:
			transformed_data = self.__reduction.transform(data)
			return transformed_data
		return data


	def get_reduction_size(self):
		''' Return number of components passed to PCA'''

		return self.__reduction_size