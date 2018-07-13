#!/usr/bin/env python

'''
ROS Anomaly Detector

Author:
	Vedanth Narayanan
File:
	Main for module
Date:
	10 May, 2018

# TODO:
#	Argument options need to be a particular order

'''

import sys
# import os
import rospy
import subprocess
from threading import Thread
from time import sleep


def call_collector(topic, output, label):
	print 'Calling collector'

	comm = "rosrun ros_anomaly_detector data_collect.py " + topic + ' ' \
				+ output + ' ' + str(label)

	listen = subprocess.Popen(comm, shell=True)
	listen.wait()


def call_train(py_script):
	print 'Going to train with passed in data'

	comm = "python " + py_script
	train = subprocess.Popen(comm, shell=True)
	train.wait()


def call_operate(topic, modelname, threshold):
	print 'Operation mode activated'

	comm = "rosrun ros_anomaly_detector monitor.py " + topic \
				+ " " + modelname + " " + threshold

	monitor = subprocess.Popen(comm, shell=True)
	monitor.wait()


if __name__ == "__main__":

	rospy.init_node('main', anonymous=True)

	print sys.argv
	# print sys.argv[1]

	if sys.argv[1] == '-collect':
		# os.system('rosrun anomaly_detector data_collect.py collecting.csv')
		print '-collect called'

		# NOTE: If a label is not passed, 0 is automatically passed
		lab = sys.argv[4] #if type(sys.argv[4]) is int else 0

		th = Thread(target=call_collector, args=(sys.argv[2], \
					sys.argv[3]), kwargs={"label": lab})
		th.start()

		th.join()

		# rospy.spin()

	if sys.argv[1] == '-train':
		print '-train called'
		th = Thread(target=call_train, kwargs={"py_script": sys.argv[2]})
		th.start()
		th.join()


	if sys.argv[1] == '-operate':
		print '-operate called'
		# TODO: BUG: Need to attach argument for specifying
		#			classifier model
		th = Thread(target=call_operate, args = (sys.argv[2], sys.argv[3], sys.argv[4]))
		th.start()

		th.join()
		# rospy.spin()
