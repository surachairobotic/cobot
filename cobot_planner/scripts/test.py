import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation2 as lib_eq


t_start = time.time()
lib_eq.create_symbol()
print('Create symbols : %f s' % (time.time()-t_start))
t_start = time.time()
lib_eq.create_equation()
print('Create eq : %f s' % (time.time()-t_start))

'''
t_start = time.time()
lib_eq.save('eq.pkl')
print('Save eq : %f s' % (time.time()-t_start))
t_start = time.time()
'''
