import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq

def run(joint_num):
  t_start = time.time()
  lib_eq.create_symbol(joint_num)
  print('Create symbols : %f s' % (time.time()-t_start))
  t_start = time.time()
  lib_eq.create_equation()
  print('Create eq : %f s' % (time.time()-t_start))
  t_start = time.time()
  lib_eq.save('eq.pkl')
  print('Save eq : %f s' % (time.time()-t_start))
  t_start = time.time()

if __name__ == "__main__":
  run(6)
