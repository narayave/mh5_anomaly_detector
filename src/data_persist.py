'''
ROS Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Data Persistence class
Date:
	15 May, 2018

NOTE:
	- Data persistence involves taking modified data and storing it.
	- That data can easily be retrieved with this class.

'''

import numpy as np


class DataPersist(object):


	def __init__(self):
		pass


	def __store_data_loc(self, name):
		'''
			Given a name of a file, returns location and name of where
				the modified data is stored and retrieved from.
			Used both of dumping and retrieving.
		'''

		name = name.split('.')[0].split('/')[1]
		loc = "./mod_data/"+name+".npz"

		return loc


	def dump_data(self, name, x, y):
		'''
			Given name of csv file, x and y data is stored in the
				mod_data/ directory.
		'''

		loc = self.__store_data_loc(name)
		np.savez(open(loc, 'w+'), x=x, y=y)

		print 'Saved to', loc


	def retrieve_dumped_data(self, name):
		'''
			Given location and file, file data is retrieved.
		'''

		loc = self.__store_data_loc(name)
		data = np.load(loc)
		x = data['x']
		y = data['y']
		data.close()

		return x, y
