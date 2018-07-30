#!/usr/bin/env python

import csv
import pandas as pd
import os

'''
	Counts
'''
def split_counts(task_counters):
	counts_arr = []
	prev = task_counters[0]
	count = 0
	for i in task_counters:
		if prev == i:
			count += 1
		else:
			counts_arr.append(count)
			count = 1
			prev = i
	counts_arr.append(count)

	return counts_arr


'''
	Reads csv first, and then spits out to csv
'''
def read(input_file):

	dataframe = pd.read_csv(input_file, header=None, engine='python')
	dataset = dataframe.values
	print dataset

	task_counters = dataset[:, 7]

	print task_counters
	counts = split_counts(task_counters)

	print counts

	directory = input_file.split('.')[0]

	if not os.path.exists(directory):
		os.mkdir(directory)

	for i in xrange(0, len(counts)):

		name = directory + '/' + str(i+1)+'.csv'

		if i == 0:
			start = 0
			end = counts[i]
		else:
			start = end
			end += counts[i]

		dataframe[start:end].to_csv(name, header=False, index=False)


if __name__ == '__main__':

	input_file = 'test.csv'
	read(input_file)

	print 'Done'