'''
Ensemble Anomaly Detector Framework

Author:
	Vedanth Narayanan
File:
	Output Matrix class
Date:
	8 May, 2018

'''

import numpy as np
from sklearn.metrics import precision_score
from sklearn.metrics import recall_score
from sklearn.metrics import f1_score


class OutputMatrix(object):

	def __init__(self):
		self.__tp = None
		self.__fp = None
		self.__tn = None
		self.__fn = None

		self.__acc = None
		self.__rec = None
		self.__prec = None
		self.__f1 = None


	def set_confusion_matrix(self, tp, fp, tn, fn):
		'''
			First to be called. All stats based on these values.
		'''
		self.__tp = tp
		self.__fp = fp
		self.__tn = tn
		self.__fn = fn


	def set_measures(self):
		'''
			Before measuers are set, make sure confusion matrix
				exists, or else exit.
		'''

		if self.__tp is None or self.__fp is None or \
			self.__tn is None or self.__fn is None:
			print 'Error: Confusion matrix values not set.'
			return

		self.set_accuracy()
		self.set_recall()
		self.set_precision()
		self.set_f1measure()


	def get_measures(self):
		'''
			Measures are printed and returned.
		'''
		acc = self.get_accuracy()
		rec = self.get_recall()
		prec = self.get_precision()
		f1 = self.get_f1measure()

		print '\taccuracy', acc
		print '\trecall', rec
		print '\tprecision', prec
		print '\tf1measure', f1

		return acc, rec, prec, f1


	def get_accuracy(self):
		return self.__acc


	def get_recall(self):
		return self.__rec


	def get_precision(self):
		return self.__prec


	def get_f1measure(self):
		return self.__f1


	def set_accuracy(self):
		res = float(self.__tp + self.__tn) / \
				(self.__tp + self.__fp + self.__tn + self.__fn + 1e-6)
		self.__acc = res


	def set_recall(self):
		res = float(self.__tp + 1e-6) / (self.__tp + self.__fn + 1e-6)
		self.__rec = res


	def set_precision(self):
		res = float(self.__tp + 1e-6) / (self.__tp + self.__fp + 1e-6)

		self.__prec = res


	def set_f1measure(self):
		res = float(2*self.__rec*self.__prec) / (self.__rec+self.__prec)
		self.__f1 = res


	def output_matrix(self, true_y, pred_y):
		'''
		Output matrix
			Meant to print the output confusion matrix. Specifically
				- True Positive
				- False Positive
				- True Negative
				- False Negative

		The above gets printed for both the training data and testing data.
		'''

		count_train = len(true_y)
		tpos = fpos = tneg = fneg = 0
		for i in xrange(count_train):
			if (pred_y[i] == 1) and (true_y[i, 0] == 1):
				tpos += 1
			elif (pred_y[i] == 1) & (true_y[i, 0] == 0):
				fpos += 1
			elif (pred_y[i] == 0) & (true_y[i, 0] == 0):
				tneg += 1
			elif (pred_y[i] == 0) & (true_y[i, 0] == 1):
				fneg += 1


		benign = float(fneg + tneg) / count_train
		attack = float(tpos + fpos) / count_train

		tpos = float(tpos) / count_train
		fpos = float(fpos) / count_train
		tneg = float(tneg) / count_train
		fneg = float(fneg) / count_train


		self.set_confusion_matrix(tpos, fpos, tneg, fneg)
		self.set_measures()

		# print 'Confusion Matrix - '
		# print '\ttpos: \t', "{:.4f}".format(tpos)
		# print '\tfpos: \t', "{:.4f}".format(fpos)
		# print '\ttneg: \t', "{:.4f}".format(tneg)
		# print '\tfneg: \t', "{:.4f}".format(fneg)
		# # print '\tleft: ', "{:.3f}".format(left)
		# print '\taccuracy: \t', "{:.4f}".format(self.get_accuracy())
		# print '\trecall: \t', "{:.4f}".format(self.get_recall())
		# print '\tprecision: \t', "{:.4f}".format(self.get_precision())
		# print '\tf1measure: \t', "{:.4f}".format(self.get_f1measure())

		print 'Benign/Attack:\t', "{:.4f}".format(benign), '\t', "{:.4f}".format(attack)
		if attack > 0.22:
			print '\tDETECT'
		else:
			print '\tBenign'

		print ''