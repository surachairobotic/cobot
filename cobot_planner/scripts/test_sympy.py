import sympy as sp
import numpy as np
import math


# set up symbols 

xyz = ['x','y','z']

t = sp.Symbol('t')  # time        (1)
g = sp.Symbol('g')  # g           (1)
q = []      # angle               (1)
L = []      # link tip's position (3x1)
COM = []    # center of mass      (4x1)
M = []      # mass                (1)
I = []      # inertia             (3x3)
Fr = []     # friction            (1)
for i in range(7):
  l = []
  c = []
  m = []
  i2 = []
  for j in range(3):
    l.append(sp.Symbol('L'+str(i) + xyz[j]))
    c.append(sp.Symbol('COM'+str(i) + xyz[j]))
    
    i3 = []
    for k in range(3):
      i3.append(sp.Symbol('I'+str(i) + '_' + str(j) + str(k)))
    i2.append(i3)
    
  c.append(1)

  if i<6:
    q.append(sp.Function('q'+str(i))(t))
    Fr.append(sp.Symbol('Fr' + str(i)))
  else:
    q.append(sp.Symbol('q'+str(i)))
    Fr.append(sp.Float(0))
  L.append(l)
  COM.append(c)
  M.append(sp.Symbol('M' + str(i)))
  I.append(i2)


#### my func ####

'''
def diff_mat(mat, t):
  v = []
  s = mat.shape
  for j in range(s[0]):
    if s[1]==1:
      v.append( sp.diff(mat[j], t) )
    else:
      v2 = []
      for i in range(s[1]):
        v2.append( sp.diff(mat[j,i], t) )
      v.append(v2)
  return sp.Matrix(v)




def trigsimp_mat(mat):
  v = []
  s = mat.shape
  for j in range(s[0]):
    if s[1]==1:
      v.append( sp.trigsimp(mat[j]) )
    else:
      v2 = []
      for i in range(s[1]):
        v2.append( sp.trigsimp(mat[j,i]) )
      v.append(v2)
  return sp.Matrix(v)
'''

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
  
'''
def Rx(q):
  return sp.Matrix([
    [1, 0, 0, 0],
    [0, sp.cos(q), -sp.sin(q), 0],
    [0, sp.sin(q), sp.cos(q), 0],
    [0, 0, 0, 1]
  ])
  
def Ry(q):
  return sp.Matrix([
    [sp.cos(q), 0, sp.sin(q), 0],
    [0, 1, 0, 0],
    [-sp.sin(q), 0, sp.cos(q), 0],
    [0, 0, 0, 1]
  ])
  
  
def Rz(q):
  return sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q) , 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ])


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
I = [
  np.diag([1,1,1]),
  np.diag([1,1,1]),
  np.diag([1,1,1]),
  np.diag([1,1,1]),
  np.diag([1,1,1]),
  np.diag([1,1,1]),
  np.diag([1,1,1])
]

# friction
Fr = [ 1.0
 , 1.0
 , 1.0
 , 1.0
 , 1.0
 , 1.0
]
'''

#### cal R ####


Rq = []

Rq.append( Rz(q[0], L[0] ) )
Rq.append( Ry(q[1], L[1] ) )
Rq.append( Ry(q[2], L[2] ) )
Rq.append( Rx(q[3], L[3] ) )
Rq.append( Ry(q[4], L[4] ) )
Rq.append( Rx(q[5], L[5] ) )
Rq.append( Rx(q[6], L[6] ) )

'''
Rq.append(sp.Matrix([
  [sp.cos(q[0]), -sp.sin(q[0]), 0, 0],
  [sp.sin(q[0]), sp.cos(q[0]) , 0, 0],
  [0, 0, 1, 0],
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

'''

R = []
r = sp.Matrix(np.diag([1,1,1,1]))

for i in range(len(Rq)):
  r = r*Rq[i]
  R.append(r)
  

#### U ####

U = 0
com = []

for i in range(len(R)):
  v1 = R[i]*sp.Matrix(COM[i])
  com.append(v1)
  v2 = (v1).dot(sp.Matrix([0,0,1,0]))
  v3 = v2*g
  U+= v3

#### T ####

T = sp.Float(0)
v_com = []
w_com = []
for i in range(len(R)):
  v1 = apply_mat( com[i], sp.diff, t )
  
#  v1 = diff_mat(com[i], t)
#  v1 = trigsimp_mat(v1)
  v_com.append(v1)
  R2 = R[i][0:3, 0:3]
  omega = apply_mat(R2, sp.diff, t) * R2.T
#  omega = diff_mat(R2, t) * R2.T
  w = sp.Matrix([omega[2,1], omega[0,2], omega[1,0]])
  w = apply_mat( w, sp.trigsimp )
#  w = trigsimp_mat(w)
  w_com.append(w)
  T+= sp.Rational(1,2)*M[i]*(v_com[i].dot(v_com[i]))
  I2 = sp.Matrix(I[i])
  T2 = (w.T * R2 * I2 * R2.T * w)[0]

  print('expand')
  T2 = sp.expand(T2)
  print('trig')
  T2 = sp.trigsimp(T2)
  print('simp')
  T2 = sp.simplify(T2)
  
  T+= sp.Rational(1,2) * T2
  
  with open('t3.txt', 'at') as f:
    f.write('\n\n------- ' + str(i) + ' -------\n\n')
    f.write('\n\n--- v ---\n\n' + str(v1))
    f.write('\n\n--- w ---\n\n' + str(w))
    f.write('\n\n--- T ---\n\n' + str(T))
  print('T : ' + str(i))
  if i==1:
    break
  
exit()
#  T+= sp.Rational(T2,2)

'''
print('simplify ...')
T = sp.simplify(sp.trigsimp(sp.expand(T)))
U = sp.simplify(sp.trigsimp(sp.expand(U)))
print('write ...')
with open('TU2.txt', 'wt') as f:
  f.write('--- T ---\n\n' + str(T) + '\n\n--- U ---\n\n' + str(U))
'''

print('eq ...')

exit()

#### kinectic equation ####

T2 = []
U2 = []

for i in range(len(R)):
#  t2 = sp.diff( T ,sp.Derivative(q[i], t))
  t2 = diff_subs( T ,sp.Derivative(q[i], t))
  T2.append( sp.diff(t2, t) )
#  U2.append( sp.diff( U, q[i] ) )
  U2.append( diff_subs( U, q[i] ) )
T2 = sp.Matrix(T2)
U2 = sp.Matrix(U2)

print('write ...')
with open('TU2.txt', 'wt') as f:
  f.write('--- t ---\n\n' + str(T2) + '\n\n--- u ---\n\n' + str(U2))



'''
#### Tq ####
Tq = sp.Float(0)
v_com = []
for i in range(len(R)):
  v = [] #sp.Matrix([0,0,0,0])
  for j in range(len(com[i])):
    v.append( sp.diff(com[i][j], t) )
  v1 = sp.Matrix(v)
  #v1 = sp.diff(com[i], t)
  v_com.append(v1)
  Tx+= sp.Float(sp.Rational(M[i],2))*(v_com[i].dot(v_com[i]))
'''

