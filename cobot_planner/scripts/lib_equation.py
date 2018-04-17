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
COM = []    # position of center of mass from i joint    (4x1)
M = []      # mass                (1)
I = []      # inertia             (3x3)
Fr = []     # friction            (1)
w0 = []     # angular veclocity in vector around rotation axis (3x1)
#eq = sp.Matrix([0]*6) # equation of motion


Rq = []         # rotation matrix from i-1 to i link    (4x4)
R = []          # rotation matrix from world to i link    (4x4)
dR = []         # dR/dt    (4x4)
dR_dq = []      # dR/dq    (4x4)
com = []        # position of COM from world   (4x1)
w_com = []      # angular velocity links       (3x1)
dw_com = []     # d(w_com)/dt   (3x1)
Kw_com = []     # Kw*dq = w_com (3x3)
dKw_com = []    # d(Kw)/dt
dKw_dq_com = [] # d(Kw)/dq
J_com = []      # Jacobian , dx = J_com * dq    (3x6)
#dJ_com = []  # dJ_com = sum( dJ_dq_com * dq )
dJ_dq_com = []  # d(J_com)/dq     # (3x6x6)
dU_dq = []      # d(U)/dq , U : potential energy

def create_symbol():
  global xyz, t, g, q, dq, ddq, L, COM, M, I, Fr, w0
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


def create_equation():
  global xyz, t, g, q, dq, ddq, L, COM, M, I, Fr, w0
  global Rq,R,dR,dR_dq,com,w_com,dw_com,Kw_com,dKw_com,dKw_dq_com,J_com, dJ_dq_com, dU_dq
  #### cal R ####
  r = sp.Matrix(np.diag([1,1,1,1]))
  Rq.append( lib_sp.Rz(q[0], L[0] ) )
  Rq.append( lib_sp.Ry(q[1], L[1] ) )
  Rq.append( lib_sp.Ry(q[2], L[2] ) )
  Rq.append( lib_sp.Rx(q[3], L[3] ) )
  Rq.append( lib_sp.Ry(q[4], L[4] ) )
  Rq.append( lib_sp.Rx(q[5], L[5] ) )
#Rq.append( lib_sp.Rx(q[6], L[6] ) )

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
  for i in range(len(R)):
    v1 = R[i]*sp.Matrix(COM[i])
    com.append(v1)
  #  v2 = (v1).dot(sp.Matrix([0,0,1,0]))
    U+= v1[2]*g*M[i]
  for i in range(len(q)):
    dU_dq.append(lib_sp.diff_subs( U, q[i]) )

  #### T ####

  T = sp.Float(0)
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

def set_const(_L,_COM,_M,_I):
  global L, COM, M, I
  global J_com, dJ_dq_com, R, dR_dq, Kw_com, dKw_dq_com
  sub = []
  for i in range(len(L)):
    for j in range(len(L[i])):
      sub.append([L[i][j], _L[i][j]]])
  for i in range(len(COM)):
    for j in range(len(COM[i])):
      sub.append([COM[i][j], _COM[i][j]])
  J_com = J_com.subs(sub)
  dJ_dq_com = dJ_dq_com.subs(sub)
  R = R.subs(sub)
  dR_dq = dR_dq.subs(sub)
  Kw_com = Kw_com.subs(sub)
  dKw_dq_com = dKw_dq_com.subs(sub)

  M = _M
  I = []
  for i in _I:
    I.append(sp.Matrix(i))



def save(file_name):
  global Rq,R,dR,dR_dq,com,w_com,dw_com,Kw_com,dKw_com,dKw_dq_com,J_com, dJ_dq_com
  lib_sp.save(file_name, [J_com, dJ_dq_com
    , Kw_com, dKw_dq_com
    , R, dR_dq], q)

def load(file_name):
  global Rq,R,dR,dR_dq,com,w_com,dw_com,Kw_com,dKw_com,dKw_dq_com,J_com, dJ_dq_com
  [J_com, dJ_dq_com \
    , Kw_com, dKw_dq_com \
    , R, dR_dq] = lib_sp.load(file_name, q)
