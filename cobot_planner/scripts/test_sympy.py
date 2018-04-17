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
ddq = []    # angular acc         (1)
L = []      # position of link's tip (3x1)
COM = []    # center of mass      (4x1)
M = []      # mass                (1)
I = []      # inertia             (3x3)
Fr = []     # friction            (1)
w0 = []     # angular veclocity in vector around rotation axis (3x1)
eq = sp.Matrix([0]*6) # equation of motion

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
    dq.append(sp.Derivative(q[-1], t))
    ddq.append(sp.Derivative(dq[-1], t))
  else:
#    q.append(sp.Symbol('q'+str(i)))
    Fr.append(sp.Float(0))
  L.append(l)
  COM.append(c)
  M.append(sp.Symbol('M' + str(i)))
  I.append(i2)

q = sp.Matrix(q)
dq = sp.Matrix(dq)
ddq = sp.Matrix(ddq)

w0.append(sp.Matrix([0,0,sp.Derivative(q[0], t)]))
w0.append(sp.Matrix([0,sp.Derivative(q[1], t),0]))
w0.append(sp.Matrix([0,sp.Derivative(q[2], t),0]))
w0.append(sp.Matrix([sp.Derivative(q[3], t),0,0]))
w0.append(sp.Matrix([0,sp.Derivative(q[4], t),0]))
w0.append(sp.Matrix([sp.Derivative(q[5], t),0,0]))

#### cal R ####

Rq = []

Rq.append( lib_sp.Rz(q[0], L[0] ) )
Rq.append( lib_sp.Ry(q[1], L[1] ) )
Rq.append( lib_sp.Ry(q[2], L[2] ) )
Rq.append( lib_sp.Rx(q[3], L[3] ) )
Rq.append( lib_sp.Ry(q[4], L[4] ) )
Rq.append( lib_sp.Rx(q[5], L[5] ) )
#Rq.append( lib_sp.Rx(q[6], L[6] ) )


R = []
dR = []
dR_dq = []
r = sp.Matrix(np.diag([1,1,1,1]))

for i in range(len(Rq)):
  r = r*Rq[i]
  R.append(r)
  dR_dq.append([])
  dR2 = sp.Matrix(np.zeros([4,4]))
  for j in range(len(Rq)):
    dR_dq[i].append( lib_sp.apply_mat( r, lib_sp.diff_subs, q[j]) )
    dR2+= dR_dq[i][-1]*dq[j]
  dR.append( dR2 )
#  dR.append( lib_sp.apply_mat( r, sp.diff, t) )
#  print(sp.simplify(dR[-1] - dR2))

#### U ####

U = 0
com = []

for i in range(len(R)):
  v1 = R[i]*sp.Matrix(COM[i])
  com.append(v1)
#  v2 = (v1).dot(sp.Matrix([0,0,1,0]))
  U+= v1[2]*g

#### T ####

T = sp.Float(0)
v_com = []
w_com = []
dw_com = []
Kw_com = [] # Kw*dq = w
dKw_com = []
dKw_dq_com = []
J_com = []
#dJ_com = []  # dJ = sum( dJ_dq * dq )
dJ_dq_com = []
t_start = time.time()

for i in range(len(R)):
  t1 = time.time()
  v = lib_sp.apply_mat( com[i], sp.diff, t )
  #### J ####
  J = lib_sp.coeff_mat(v, dq)
#  J = sp.simplify(J)
  J_com.append(J)

  #### dJ ####
  dJ_dq_com.append([])
  for j in range(len(R)):
    dJ_dq_com[i].append( lib_sp.apply_mat( J, lib_sp.diff_subs, q[j]) )
#  dJ = lib_sp.apply_mat( J, sp.diff, t)
#  dJ_com.append(dJ)

#  eq+= M[i]*(dJ.T*J*dq + J.T*dJ*dq + J.T*J*ddq)

  #### w ####
  # w'n = w1 + Rq1*(w2 + Rq2*(w3+...))
  if i==0:
    w_com.append(w0[i])
  else:
    w_com.append( w_com[-1] + R[i-1][0:3,0:3]*w0[i] )
  dw_com.append( lib_sp.apply_mat( w_com[-1], sp.diff, t) )
  Kw_com.append( lib_sp.coeff_mat(w_com[-1], dq) )

  dKw_dq_com.append([])
  dKw2 = sp.Matrix(np.zeros(Kw_com[-1].shape))
  for j in range(len(q)):
    dKw_dq_com[i].append( lib_sp.diff_subs(Kw_com[-1], q[j]) )
    dKw2+= dKw_dq_com[i][-1]*dq[j]
  dKw_com.append( dKw2 )
#  dKw_com.append( lib_sp.apply_mat( Kw_com[-1], sp.diff, t) )

  # eq+= d/dt( Kw.T*R*I*R.T*dq )
  # dR*I*Rt*w + R*I*dRt*w + R*I*Rt*dw
#  eq+= dR[i]*I[i]*R[i].T*w_com[i] + R[i]*I[i]*dR[i].T*w_com[i] + R[i]*I[i]*R[i].T*dw_com[i]
  '''
  eq+= dKw_com[i]*R[i]*I[i]*R[i].T*Kw_com[i]*dq
    + Kw_com[i]*dR[i]*I[i]*R[i].T*Kw_com[i]*dq
    + Kw_com[i]*R[i]*I[i]*dR[i].T*Kw_com[i]*dq
    + Kw_com[i]*R[i]*I[i]*R[i].T*dKw_com[i]*dq
    + Kw_com[i]*R[i]*I[i]*R[i].T*Kw_com[i]*ddq
  '''

  #### dT/dq ####
  # m*dq*dJ_dq.T*J*dq

  print('t[%d] : %f' % (i, (time.time() - t1)))
print(time.time() - t_start)
#lib_sp.save('eq.pkl', [eq], q)
#lib_sp.save_text('eq.txt', dJ_com)
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
