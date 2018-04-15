import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp

# set up symbols

xyz = ['x','y','z']

t = sp.Symbol('t')  # time        (1)
g = sp.Symbol('g')  # g           (1)
q = []      # angle               (1)
dq = []     # angular velocity    (1)
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
  dq.append(sp.Derivative(q[-1], t))
  L.append(l)
  COM.append(c)
  M.append(sp.Symbol('M' + str(i)))
  I.append(i2)

#### cal R ####

Rq = []

Rq.append( lib_sp.Rz(q[0], L[0] ) )
Rq.append( lib_sp.Ry(q[1], L[1] ) )
Rq.append( lib_sp.Ry(q[2], L[2] ) )
Rq.append( lib_sp.Rx(q[3], L[3] ) )
Rq.append( lib_sp.Ry(q[4], L[4] ) )
Rq.append( lib_sp.Rx(q[5], L[5] ) )
Rq.append( lib_sp.Rx(q[6], L[6] ) )


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
J_com = []
dJ_com = []
t_start = time.time()
for i in range(len(R)):
  t1 = time.time()
  v = lib_sp.apply_mat( com[i], sp.diff, t )

#  J = sp.simplify(lib_sp.coeff_mat(v, dq))
#  dJ = sp.simplify(lib_sp.apply_mat( J, sp.diff, t))
  J = lib_sp.coeff_mat(v, dq)
  dJ = lib_sp.apply_mat( J, sp.diff, t)
  J_com.append(J)
  dJ_com.append(dJ)
  print('t[%d] : %f' % (i, (time.time() - t1)))
  '''
  if i==1:
    lib_sp.save('test.pkl', [J_com, dJ_com], q)
    J_com2, dJ_com2 = lib_sp.load('test.pkl', q)
    lib_sp.save_text('J_com.txt', J_com)
    exit()
  '''

print(time.time() - t_start)
lib_sp.save('test.pkl', [J_com, dJ_com], q)
lib_sp.save_text('J_com.txt', J_com)
lib_sp.save_text('dJ_com.txt', dJ_com)
exit()

if 0:



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
