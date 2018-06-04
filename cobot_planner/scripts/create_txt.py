import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq

def run(joint_num,b_load=False):
  if b_load:
    t_start = time.time()
    lib_eq.create_symbol(joint_num)
    print('Create symbols : %f s' % (time.time()-t_start))

    t_start = time.time()
    lib_eq.load('eq_const.pkl')
    print('Load data : %f s' % (time.time()-t_start))

  t_start = time.time()
  lib_eq.save_txt('eq.py')
  print('Save txt : %f s' % (time.time()-t_start))

if __name__ == "__main__":
  run(6,True)
