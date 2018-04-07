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



def diff_mat(mat):
  v = []
  s = mat.shape
  for j in range(s[0]):
    if s[1]==1:
      v.append( sp.diff(mat[j,i], t) )
    else:
      v2 = []
      for i in range(s[1]):
        v2.append( sp.diff(mat[j,i], t) )
      v.append(v2)
  return sp.Matrix(v)


R = R[-1][0:3, 0:3]
omega = diff_mat(R) * R.T

w = omega.subs( sp.Derivative(q[0], t), 1 )
w = omega

for i in range(6):
  w = w.subs( sp.Derivative(q[i], t), i+1 )
  w = w.subs( q[i], (i+1)*0.1 )

print(w)

print('w : ' + str([w[2,1], w[0,2], w[1,0]]))

# I' = R*I*R.T
# inertia = 1/2*w*I'*w

'''
w = omega.subs( [ sp.Derivative(q[0], t), 1
  , sp.Derivative(q[1], t), 2
  , sp.Derivative(q[2], t), 3
  , sp.Derivative(q[3], t), 4
  , sp.Derivative(q[4], t), 5
  , sp.Derivative(q[5], t), 6
  , q[0], 0.1
  , q[1], 0.2
  , q[2], 0.3
  , q[3], 0.4
  , q[4], 0.5
  , q[5], 0.6]
)
print(w)
'''


