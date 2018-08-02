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

TASK_COUNT = 0


def task_count(val):
	global TASK_COUNT

	print val.data

	TASK_COUNT = int(val.data)

'''
	Callback function when a joint_states message is published.
	Adds positions to global list to be processed right afterwards.
'''
def control_command(val):
	global POSITIONS_LIST
	global LABEL
	global PREV
	global TASK_COUNT

	if list(val.position) == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
		return

	if LABEL:
		if PREV != list(val.position):
			POSITIONS_LIST.append(list(val.position) + [LABEL] + [TASK_COUNT])
			PREV = list(val.position)
	else:
		if PREV != list(val.position):
			POSITIONS_LIST.append(list(val.position) + [TASK_COUNT])
			PREV = list(val.position)


'''
	Helper for writing positions to csv
'''
def collect(out):

	while POSITIONS_LIST:
		out.writerow(POSITIONS_LIST[0])
		POSITIONS_LIST.pop(0)


if __name__ == '__main__':

	print 'collect - ', sys.argv

	rospy.init_node('data_collector', anonymous=True)
	rospy.Subscriber(sys.argv[1],
				JointState,
				control_command)

	rospy.Subscriber('task_counter',
					String,
					task_count)


	# Argv 1 - path + file name is expected
	cwd = sys.argv[2]

	# NOTE: The label is an option
	if len(sys.argv) > 3:
		LABEL = sys.argv[3]

	sleep(3)
	with open(cwd, "w+") as out_file:
		out = csv.writer(out_file, delimiter=',')
		print 'Writing to', cwd
		while not rospy.is_shutdown():
			collect(out)
