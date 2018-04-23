import sympy as sp
import numpy as np
import math
import pickle

simp_cs = None
#simp_q = None
_subs_q = None
_desubs_q = None

def diff_subs( val, d ):
  tmp = sp.Symbol('tmp')
  diff = val.subs( d, tmp )
  diff = sp.diff( diff, tmp )
  return diff.subs( tmp, d )


def apply_mat(mat, func, var1=None):
  v = []
  s = mat.shape
  for j in range(s[0]):
    if s[1]==1:
      if var1 is None:
        v.append( func(mat[j]) )
      else:
        v.append( func(mat[j], var1) )
    else:
      v2 = []
      for i in range(s[1]):
        if var1 is None:
          v2.append( func(mat[j,i]) )
        else:
          v2.append( func(mat[j,i], var1) )
      v.append(v2)
  return sp.Matrix(v)

def apply_mat_recursive(func, mat, var1=None, var2=None):
  if type(mat)==list:
    v2 = []
    for i in range(len(mat)):
      v2.append(apply_mat_recursive( func, mat[i], var1, var2) )
    return v2
  else:
    if var1 is None:
      return func(mat)
    elif var2 is None:
      return func(mat,var1)
    else:
      return func(mat,var1,var2)

def simplify_cs(val, q):
  global simp_cs
  v = val
  if simp_cs is None:
    simp_cs = [[],[]]
    for i in range(len(q)):
      simp_cs[0].append(sp.Symbol('c'+str(i)))
      simp_cs[1].append(sp.Symbol('s'+str(i)))
  for i in range(len(q)):
    v = v.subs(sp.cos(q[i]), simp_cs[0][i])
    v = v.subs(sp.sin(q[i]), simp_cs[1][i])
  return sp.simplify(v)


def subs_mat(mat, s1, s2=None):
  if not hasattr(mat, 'shape'):
    return mat
  s = mat.shape
  mat2 = sp.Matrix(np.zeros([s[0],s[1]]))
  for j in range(s[0]):
    for k in range(s[1]):
      mat2[j,k] = mat[j,k]
      if s2 is None:
        mat2[j,k] = mat2[j,k].subs(s1)
      else:
        if len(s1)==1:
          mat2[j,k] = mat2[j,k].subs(s1, s2)
        else:
          for i in range(len(s1)):
            mat2[j,k] = mat2[j,k].subs(s1[i], s2[i])
  return mat2


'''
def subs_mat(mat, s1, s2):
  s = mat.shape
  mat2 = sp.Matrix(np.zeros([s[0],s[1]]))
  for j in range(s[0]):
    if s[1]==1:
      mat2[j] = mat[j]
      if len(s1)==1:
        mat2[j] = mat2[j].subs(s1, s2)
      else:
        for i in range(len(s1)):
          mat2[j] = mat2[j].subs(s1[i], s2[i])
    else:
      for k in range(s[1]):
        mat2[j,k] = mat[j,k]
        if len(s1)==1:
          mat2[j,k] = mat2[j,k].subs(s1, s2)
        else:
          for i in range(len(q)):
            mat2[j,k] = mat2[j,k].subs(s1[i], s2[i])
  return mat2
'''




def Rx(q, L):
  return sp.Matrix([
    [1, 0, 0, L[0]],
    [0, sp.cos(q), -sp.sin(q), L[1]],
    [0, sp.sin(q), sp.cos(q) , L[2]],
    [0, 0, 0, 1]
  ])

def Ry(q, L):
  return sp.Matrix([
    [sp.cos(q), 0, sp.sin(q), L[0]],
    [0, 1, 0, L[1]],
    [-sp.sin(q), 0, sp.cos(q) , L[2]],
    [0, 0, 0, 1]
  ])


def Rz(q, L):
  return sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, L[0]],
    [sp.sin(q), sp.cos(q) , 0, L[1]],
    [0, 0, 1, L[2]],
    [0, 0, 0, 1]
  ])

'''
def recursive_simp(simp, var, q):
  if type(var)==list:
    var2 = [None]*len(var)
    for i in range(len(var)):
      var2[i] = recursive_simp(simp, var[i], q)
    return var2
  else:
    return simp(var, q)
'''


'''
def simplify_q(mat, q):
  global simp_q
  if simp_q is None:
    simp_q = []
    for i in range(len(q)):
      simp_q.append(sp.Symbol('qt'+str(i)))
  return subs_mat(mat, q, simp_q )

def desimplify_q(mat, q):
  global simp_q
  if simp_q is None:
    simp_q = []
    for i in range(len(q)):
      simp_q.append(sp.Symbol('qt'+str(i)))
  return subs_mat(mat, simp_q, q )
'''


def create_subs_q():
  global _subs_q
  _subs_q = []

def subs_q(mat, q):
  global _subs_q
  '''
  if _subs_q is None:
    _subs_q = []
    for i in range(len(q)):
      _subs_q.append(sp.Symbol('qt'+str(i)))
  return subs_mat(mat, q, _subs_q )
  '''
  if _subs_q is None:
    _subs_q = []
    for i in range(len(q)):
      _subs_q.append([ sp.cos(q[i]),  sp.Symbol('c'+str(i)) ])
      _subs_q.append([ sp.sin(q[i]),  sp.Symbol('s'+str(i)) ])
      _subs_q.append([ sp.Derivative(q[i], sp.Symbol('t')), sp.Symbol('dq'+str(i)) ])
  return subs_mat(mat, _subs_q )

def desubs_q(mat, q):
  global _subs_q
  if _subs_q is None:
    _subs_q = []
    for i in range(len(q)):
      _subs_q.append(sp.Symbol('qt'+str(i)))
  return subs_mat(mat, _subs_q, q )

def save(file_name, var, q):
  v = apply_mat_recursive( subs_q, var, q )
#  v = recursive_simp(simplify_q, var, q)
  with open(file_name, 'wb') as f:
    pickle.dump(v, f)

def load(file_name, q):
  with open(file_name, 'rb') as f:
    var = pickle.load(f)
#    v = apply_mat_recursive( desubs_q, var, q)
#    v = recursive_simp(desimplify_q, var, q)
    return var

def save_text(file_name, var):
  with open(file_name, 'wt') as f:
    f.write(str(var))

def coeff_mat(var, q):
  s = var.shape
  assert(s[1]==1)
  v = sp.Matrix(np.zeros([s[0], len(q)]))
  for j in range(s[0]):
    for k1 in range(len(q)):
      c = var[j]
      for k2 in range(len(q)):
        if k1==k2:
          n = 1
        else:
          n = 0
        c = c.subs(q[k2], n)
      v[j,k1] = c
  v2 = sp.simplify(v*q - var).doit()
  for i in range(v2.shape[0]):
    if v2[i]!=0:
      print(v)
      print(q)
      print(var)
      print(v2)
      assert(v2[i]==0)
  return v
