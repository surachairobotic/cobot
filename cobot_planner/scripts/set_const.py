import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq


L = [[0,0,0]
,[0,0,0.4]
,[0,0,0.3]
,[0,0,0.2]
,[0,0,0.1]
,[0,0,0.05]
,[0,0,0.08]
]

COM = [[0,0,0]
,[0,0,0.2]
,[0,0,0.1]
,[0,0,0.12]
,[0,0,0.05]
,[0,0,0.03]
,[0,0,0.04]
]

M = [0,3,2,2.2,1.5,1.2,0.8,0.7]
I = [np.diag([2,2,1])
, np.diag([1,1,2])
, np.diag([1,2,1])
, np.diag([1,2,2])
, np.diag([2,1,1])
, np.diag([1,2,1])
, np.diag([2,2,1])
]

t_start = time.time()
lib_eq.create_symbol()
print('Create symbols : %f s' % (time.time()-t_start))

t_start = time.time()
lib_eq.load('eq.pkl')
print('Load data : %f s' % (time.time()-t_start))

t_start = time.time()
lib_eq.set_const(L,COM,M,I)
print('Set constant : %f s' % (time.time()-t_start))

t_start = time.time()
lib_eq.save('eq_const.pkl')
print('Save eq : %f s' % (time.time()-t_start))
