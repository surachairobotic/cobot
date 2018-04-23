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
L_org = []  # pos from world to link-0  (3x1)
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
w = []      # angular velocity links       (3x1)
dw = []     # d(w)/dt   (3x1)
Kw = []     # Kw*dq = w (3x3)
dKw = []    # d(Kw)/dt
dKw_dq = [] # d(Kw)/dq
J = []      # Jacobian , dx = J * dq    (3x6)
dJ = []     # dJ = sum( dJ_dq * dq )
dJ_dq = []  # d(J)/dq     (3x6x6)
com_z = []      # pos-z of each link for U    (6x1)
dz_dq = []      # d(com_z)/dq    (6x6)

def create_symbol(joint_num):
  global xyz, t, g, q, dq, ddq, L_org, L, COM, M, I, Fr, w0
  if len(q)>0:
    global R,dR,dR_dq,com,w,dw,Kw,dKw,dKw_dq,J, dJ_dq \
      ,dz_dq, com_z
    q = []      # angle               (1)
    dq = []     # angular velocity    (1)
    ddq = []    # angular acc         (1)
    L_org = []  # pos from world to link-0  (3x1)
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
    w = []      # angular velocity links       (3x1)
    dw = []     # d(w)/dt   (3x1)
    Kw = []     # Kw*dq = w (3x3)
    dKw = []    # d(Kw)/dt
    dKw_dq = [] # d(Kw)/dq
    J = []      # Jacobian , dx = J * dq    (3x6)
    dJ = []     # dJ = sum( dJ_dq * dq )
    dJ_dq = []  # d(J)/dq     (3x6x6)
    com_z = []      # pos-z of each link for U    (6x1)
    dz_dq = []      # d(com_z)/dq    (6x6)
  for i in range(joint_num):
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
  for i in range(len(xyz)):
    L_org.append(sp.Symbol('L_org'+ xyz[i]))
  q = sp.Matrix(q)
  dq = sp.Matrix(dq)
  ddq = sp.Matrix(ddq)

  try:
    w0.append(sp.Matrix([0,0,sp.Derivative(q[0], t)]))
    w0.append(sp.Matrix([0,sp.Derivative(q[1], t),0]))
    w0.append(sp.Matrix([0,sp.Derivative(q[2], t),0]))
    w0.append(sp.Matrix([sp.Derivative(q[3], t),0,0]))
    w0.append(sp.Matrix([0,sp.Derivative(q[4], t),0]))
    w0.append(sp.Matrix([sp.Derivative(q[5], t),0,0]))
  except:
    pass
  '''
  if len(w0)!=len(q):
    for i in range(len(w0)-len(q)):
      w0.pop()
  '''


def create_equation():
  global xyz, t, g, q, dq, ddq, L_org, L, COM, M, I, Fr, w0
  global Rq,R,dR,dR_dq,com,w,dw,Kw,dKw,dKw_dq,J, dJ_dq, dz_dq, com_z
  #### cal R ####
  r = sp.Matrix(np.diag([1,1,1,1]))
  try:
    Rq.append( lib_sp.Rz(q[0], L_org ) )
    Rq.append( lib_sp.Ry(q[1], L[0] ) )
    Rq.append( lib_sp.Ry(q[2], L[1] ) )
    Rq.append( lib_sp.Rx(q[3], L[2] ) )
    Rq.append( lib_sp.Ry(q[4], L[3] ) )
    Rq.append( lib_sp.Rx(q[5], L[4] ) )
    '''
    Rq.append( lib_sp.Rz(q[0], L[0] ) )
    Rq.append( lib_sp.Ry(q[1], L[1] ) )
    Rq.append( lib_sp.Ry(q[2], L[2] ) )
    Rq.append( lib_sp.Rx(q[3], L[3] ) )
    Rq.append( lib_sp.Ry(q[4], L[4] ) )
    Rq.append( lib_sp.Rx(q[5], L[5] ) )
    '''
  except:
    pass
#Rq.append( lib_sp.Rx(q[6], L[6] ) )

  for i in range(len(q)):
    r = r*Rq[i]
    R.append(r)
    dR_dq.append([])
    dR2 = sp.Matrix(np.zeros([4,4]))
    for j in range(len(q)):
      dR_dq[i].append( lib_sp.apply_mat( r, lib_sp.diff_subs, q[j]) )
      dR2+= dR_dq[i][-1]*dq[j]
    dR.append( dR2 )
  #  dR.append( lib_sp.apply_mat( r, sp.diff, t) )
  #  print(sp.simplify(dR[-1] - dR2))

  #### U ####

  U = 0
  for i in range(len(q)):
    v1 = R[i]*sp.Matrix(COM[i])
    com.append(v1)
  #  v2 = (v1).dot(sp.Matrix([0,0,1,0]))
  '''
    U+= v1[2]*g*M[i]
  for i in range(len(q)):
    dU_dq.append(lib_sp.diff_subs( U, q[i]) )
  '''

  # cal pos[2] of each link for U
  com_z = []
  for i in range(len(q)):
    com_z.append(com[i][2])
    d = []
    for j in range(len(q)):
      d.append( com_z[i].diff(q[j]) )
    dz_dq.append(d)
  com_z = sp.Matrix(com_z)
  dz_dq = sp.Matrix(dz_dq)
  #### T ####

  T = sp.Float(0)
  for i in range(len(q)):
    t1 = time.time()
    v = lib_sp.apply_mat( com[i], sp.diff, t )
    #### J ####
    J2 = lib_sp.coeff_mat(v, dq)
  #  J = sp.simplify(J)
    J.append(J2)

    #### dJ ####
    dJ_dq.append([])
    dJ2 = sp.Matrix(np.zeros(J[-1].shape))
    for j in range(len(R)):
      dJ_dq[i].append( lib_sp.apply_mat( J2, lib_sp.diff_subs, q[j]) )
      dJ2+= dJ_dq[i][-1]*dq[j]
    dJ.append(dJ2)
  #  dJ = lib_sp.apply_mat( J, sp.diff, t)
  #  dJ.append(dJ)

  #  eq+= M[i]*(dJ.T*J*dq + J.T*dJ*dq + J.T*J*ddq)

    #### w ####
    # w'n = w1 + Rq1*(w2 + Rq2*(w3+...))
    if i==0:
      w.append(w0[i])
    else:
      w.append( w[-1] + R[i-1][0:3,0:3]*w0[i] )
    dw.append( lib_sp.apply_mat( w[-1], sp.diff, t) )
    Kw.append( lib_sp.coeff_mat(w[-1], dq) )

    dKw_dq.append([])
    dKw2 = sp.Matrix(np.zeros(Kw[-1].shape))
    for j in range(len(q)):
      dKw_dq[i].append( lib_sp.diff_subs(Kw[-1], q[j]) )
      dKw2+= dKw_dq[i][-1]*dq[j]
    dKw.append( dKw2 )
  #  dKw.append( lib_sp.apply_mat( Kw[-1], sp.diff, t) )

    # eq+= d/dt( Kw.T*R*I*R.T*dq )
    # dR*I*Rt*w + R*I*dRt*w + R*I*Rt*dw
  #  eq+= dR[i]*I[i]*R[i].T*w[i] + R[i]*I[i]*dR[i].T*w[i] + R[i]*I[i]*R[i].T*dw[i]
    '''
    eq+= dKw[i]*R[i]*I[i]*R[i].T*Kw[i]*dq
      + Kw[i]*dR[i]*I[i]*R[i].T*Kw[i]*dq
      + Kw[i]*R[i]*I[i]*dR[i].T*Kw[i]*dq
      + Kw[i]*R[i]*I[i]*R[i].T*dKw[i]*dq
      + Kw[i]*R[i]*I[i]*R[i].T*Kw[i]*ddq
    '''
    #### dT/dq ####
    # m*dq*dJ_dq.T*J*dq

def set_const(_L_org, _L,_COM,_M,_I,_g):
  global L_org, L, COM, M, I
  global J, dJ_dq, R, dR_dq, Kw, dKw_dq, dz_dq, com_z, g
  sub = []
  for i in range(len(L_org)):
    sub.append([L_org[i], _L_org[i]])
  for i in range(len(L)):
    for j in range(len(L[i])):
      sub.append([L[i][j], _L[i][j]])
  for i in range(len(COM)):
    for j in range(len(_COM[i])):
      sub.append([COM[i][j], _COM[i][j]])

  '''
  for i in range(len(J)):
    J[i] = J[i].subs(sub)
    R[i] = R[i].subs(sub)
    Kw[i] = Kw[i].subs(sub)
    for j in range(len(dKw_dq[i])):
      dJ_dq[i][j] = dJ_dq[i][j].subs(sub)
      dR_dq[i][j] = dR_dq[i][j].subs(sub)
      dKw_dq[i][j] = dKw_dq[i][j].subs(sub)
  '''
  vars = [J, dJ_dq, R, dR_dq, Kw, dKw_dq, dz_dq, com_z]
  J, dJ_dq, R, dR_dq, Kw, dKw_dq, dz_dq, com_z = \
    lib_sp.apply_mat_recursive( lib_sp.subs_mat, vars, sub )
  '''
  R[i] = R[i].subs(sub)
  Kw[i] = Kw[i].subs(sub)
  for j in range(len(dKw_dq[i])):
    dJ_dq[i][j] = dJ_dq[i][j].subs(sub)
    dR_dq[i][j] = dR_dq[i][j].subs(sub)
    dKw_dq[i][j] = dKw_dq[i][j].subs(sub)
  '''

#  print(J[4])
  M = sp.Matrix(_M)
  I = []
  g = _g
  for i in _I:
    I.append(sp.Matrix(i))




def save_txt(file_name):
  global R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,com, q,dz_dq, com_z, M, I, g

  vars = [R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,dz_dq, M, I, g]
  s = 'import numpy as np\nimport math\n\n'
  s+= 'def get_vars(q):\n'
  for i in range(len(R)):
    s+= "  c"+str(i)+" = math.cos(q["+str(i)+'])\n'
    s+= "  s"+str(i)+" = math.sin(q["+str(i)+'])\n'
  s+='  return '
  for i in range(len(vars)):
    s+= str(vars[i]) + ', '

  s+= '\n\ndef get_z(q):\n'
  for i in range(len(R)):
    s+= "  c"+str(i)+" = math.cos(q["+str(i)+'])\n'
    s+= "  s"+str(i)+" = math.sin(q["+str(i)+'])\n'
  s+='  return ' + str(com_z)
  s = s.replace('Matrix(','np.array(')
#  s = s.replace('])',']')
  with open(file_name, 'wt') as f:
    f.write(s)

def save(file_name):
  global R,dR,dR_dq,com,Kw,dKw,dKw_dq,J, dJ_dq, dJ,q, M, I,dz_dq, g
  lib_sp.save(file_name, [J, dJ_dq, dJ
    , Kw, dKw_dq,dz_dq, com_z
    , R, dR_dq, M, I, g], q)

def load(file_name):
  global R,dR,dR_dq,com,Kw,dKw,dKw_dq,J, dJ_dq \
    ,q,dq,ddq, M, I,dz_dq, com_z, g
  [J, dJ_dq, dJ \
    , Kw, dKw_dq,dz_dq, com_z \
    , R, dR_dq, M, I, g] = lib_sp.load(file_name, q)
  q = lib_sp.apply_mat_recursive( lib_sp.subs_q, q, q)
  dq = lib_sp.apply_mat_recursive( lib_sp.subs_q, dq, q)
  ddq = lib_sp.apply_mat_recursive( lib_sp.subs_q, ddq, q)
