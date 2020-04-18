#!/usr/bin/python

import numpy
from scipy import interpolate
import csv
from matplotlib import pyplot

''' 
Display the Cl and Cd curves for the iceboat sail.

'''

clcda = csv.reader(open('cl-alpha.csv'))
clcda.__next__()
alpha=[]
cl=[]
for row in clcda:
    alpha.append(float(row[0]))
    cl.append(float(row[1]))
cl_alpha = interpolate.splrep(alpha, cl)
print(alpha, cl)

clcda = csv.reader(open('cd-alpha.csv'))
clcda.__next__()
alpha1=[]
cd=[]
for row in clcda:
    alpha1.append(float(row[0]))
    cd.append(float(row[1]))
cd_alpha = interpolate.splrep(alpha1, cd)

anew = range(0, 180, 2)
clnew = interpolate.splev(anew, cl_alpha)
cdnew = interpolate.splev(anew, cd_alpha)

pyplot.plot(alpha, cl, 'x', anew, clnew,
            alpha1, cd, 'o', anew, cdnew)
pyplot.show()

'''
from scipy.interpolate import interp1d
f1 = interp1d(alpha,cl)
cl_10 = f1(10)
'''