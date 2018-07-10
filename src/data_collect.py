#!/usr/bin/env python

import rospy
import sys
import os
from time import sleep
from sensor_msgs.msg import JointState
import csv
from std_msgs.msg import String


POSITIONS_LIST = []
LABEL = None
COUNT = 0
PREV = None


'''
	Callback function when a joint_states message is published.
	Adds positions to global list to be processed right afterwards.
'''
def control_command(val):
	global POSITIONS_LIST
	global LABEL
	global PREV
	global TASK_COUNT

	if LABEL:
		if PREV != list(val.position):
			POSITIONS_LIST.append(list(val.position) + [LABEL])
			PREV = list(val.position)
	else:
		if PREV != list(val.position):
			POSITIONS_LIST.append(list(val.position))
			PREV = list(val.position)


'''
	Helper for writing positions to csv
'''
def collect(out):

	while POSITIONS_LIST:
		out.writerow(POSITIONS_LIST[0])
		POSITIONS_LIST.pop(0)


if __name__ == '__main__':

	global LABEL

	print 'collect - ', sys.argv


	rospy.init_node('collecter', anonymous=True)
	rospy.Subscriber(sys.argv[1],
				JointState,
				control_command)


	# NOTE: Argv 1 - file name is expected
	location = sys.argv[2]
	cwd = os.getcwd()+'/data/'+location

	# NOTE: The label is an option
	if len(sys.argv) > 3:
		LABEL = sys.argv[3]

	sleep(3)
	with open(cwd, "w+") as out_file:
		out = csv.writer(out_file, delimiter=',')
		print 'Writing to', cwd
		while not rospy.is_shutdown():
			collect(out)
