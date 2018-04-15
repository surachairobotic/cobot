import sympy as sp
import numpy as np
import math
import pickle

simp_cs = None
simp_q = None

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



def subs_mat(mat, s1, s2):
  s = mat.shape
  mat2 = sp.Matrix(np.zeros([s[0],s[1]]))
  for j in range(s[0]):
    for k in range(s[1]):
      mat2[j,k] = mat[j,k]
      if len(s1)==1:
        mat2[j,k] = mat2[j,k].subs(s1, s2)
      else:
        for i in range(len(q)):
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


def recursive_simp(simp, var, q):
  if type(var)==list:
    var2 = [None]*len(var)
    for i in len(var):
      var2[i] = recursive_simp(var[i], q)
    return var2
  else:
    return simp(var, q)

def save(file_name, var, q):
  if type(var)==list:
    var2 = []
    for v in var:
      var2.append( simplify_q(v, q) )
    var = var2

  with open(file_name, 'wb') as f:
    pickle.dump(var, f)

def load(file_name, var, q):
  with open(file_name, 'rb') as f:
    var = pickle.load(f)
    if type(var)==list:
      var2 = []
      for v in var:
        var2.append( desimplify_q(v, q) )
      var = var2
    return var
