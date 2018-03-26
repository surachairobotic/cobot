import sympy as sp
import numpy as np
import math

g = 9.8

# set up our joint angle symbols (6th angle doesn't affect any kinematics)
t = sp.Symbol('t')
q = []
for i in range(6):
  q.append(sp.Function('q'+str(i))(t))
#q = [sp.Symbol('q%i(t)'%ii) for ii in range(6)]
# segment lengths associated with each joint
#L = np.array([1,2,3,4,5])
L = [
  [0,0,0],
  [0,0,0],
  [1,0,0],
  [2,0,0],
  [3,0,0],
  [4,0,0],
  [5,0,0]
]

# center of mass
COM = [
  [0,0,0,1],
  [0,0,0,1],
  [1,0,0,1],
  [2,0,0,1],
  [3,0,0,1],
  [4,0,0,1],
  [5,0,0,1]
]

# mass
M = [
  0,
  1,
  1,
  1,
  1,
  1,
  1
]

# inertia
I = sp.Matrix([
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1]),
  np.diag([1,1,1,1])
])

Rq = []

Rq.append(sp.Matrix([
  [sp.cos(q[0]), -sp.sin(q[0]), 0, L[0][0]],
  [sp.sin(q[0]), sp.cos(q[0]) , 0, L[0][1]],
  [0, 0, 1, L[0][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [sp.cos(q[1]), 0, sp.sin(q[1]), L[1][0]],
  [0, 1, 0, L[1][1]],
  [-sp.sin(q[1]), 0, sp.cos(q[1]), L[1][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [sp.cos(q[2]), 0, sp.sin(q[2]), L[2][0]],
  [0, 1, 0, L[2][1]],
  [-sp.sin(q[2]), 0, sp.cos(q[2]), L[2][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [1, 0, 0, L[3][0]],
  [0, sp.cos(q[3]), -sp.sin(q[3]), L[3][1]],
  [0, sp.sin(q[3]), sp.cos(q[3]), L[3][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [sp.cos(q[4]), 0, sp.sin(q[4]), L[4][0]],
  [0, 1, 0, L[4][1]],
  [-sp.sin(q[4]), 0, sp.cos(q[4]), L[4][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [1, 0, 0, L[5][0]],
  [0, sp.cos(q[5]), -sp.sin(q[5]), L[5][1]],
  [0, sp.sin(q[5]), sp.cos(q[5]), L[5][2]],
  [0, 0, 0, 1]
]))

Rq.append(sp.Matrix([
  [1, 0, 0, L[6][0]],
  [0, 1, 0, L[6][1]],
  [0, 0, 1, L[6][2]],
  [0, 0, 0, 1]
]))

R = []
r = sp.Matrix([
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
])
for i in range(len(Rq)):
  r = r*Rq[i]
  R.append(r)

U = 0
com = []
for i in range(len(R)):
  v1 = R[i]*sp.Matrix(COM[i])
  com.append(v1)
  v2 = (v1).dot(sp.Matrix([0,0,1,0]))
  v3 = v2*g
  U+= v3

#### Tx ####
Tx = sp.Float(0)
v_com = []
for i in range(len(R)):
  v1 = sp.diff(com[i], t)
  v_com.append(v1)
  Tx+= sp.Float(sp.Rational(M[i],2))*(v_com[i].dot(v_com[i]))

#### Tq ####
Tq = sp.Float(0)
v_com = []
for i in range(len(R)):
  v1 = sp.diff(com[i], t)
  v_com.append(v1)
  Tx+= sp.Float(sp.Rational(M[i],2))*(v_com[i].dot(v_com[i]))
