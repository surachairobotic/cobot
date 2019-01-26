# created by cobot_test_eq.cpp to check eq_c.h calculation

import numpy as np
import eq
from test_eq_c_param import *
import test_cal_torque


vars = [
  [ [R,dR_dq,Kw,dKw_dq,J,dJ_dq,dz_dq,M,I,g], eq.get_vars(q)],
  [ [torque_ddq, torque_no_ddq], None ],
  [ [dJ_dq, J, dR_dq[:,:,:3,:3], dKw_dq, R[:,:3,:3], I, Kw], [m_dJ_dq, m_J, m_dR_dq, m_dKw_dq, m_R, m_I, m_Kw]],
  [ [dT_dq, dU_dq, dT_ddq, K_ddq, dq, ddq ], None ]
]

tq = test_cal_torque.cal_torque(q, dq, ddq, vars[0][1])
vars[1][1] = tq[0:2]
vars[3][1] = list(tq[2:]) + [m_dq, m_ddq]

for i in range(len(vars)):
  for j in range(len(vars[i][0])):
#    print('%d, %d' % (i,j))
    e = (vars[i][0][j] - vars[i][1][j])
    if type(e) is np.ndarray:
      e = e.max()
    if abs(e)>0.00001:
      print('Error [%d,%d] : %f' % (i, j, e))
#for i in range(2):
#  print(vars[i][len(vars[i])-2:len(vars[i])])
#print(np.array(vars[0][len(vars[0])-2:len(vars[0])]) - np.array(vars[1][len(vars[1])-2:len(vars[1])]))

print('check var error OK')

'''
R2,dR_dq2,Kw2,dKw_dq2,J2, dJ_dq2,dz_dq2,M2,I2,g2 = eq.get_vars(q)
print( "R : " + str((R - R2).max()) )
print( "dR_dq : " + str((dR_dq - dR_dq2).max()) )
print( "Kw : " + str((Kw - Kw2).max()) )
print( "dKw_dq : " + str((dKw_dq - dKw_dq2).max()) )
print( "J : " + str((J - J2).max()) )
print( " dJ_dq : " + str(( dJ_dq -  dJ_dq2).max()) )
print( "dz_dq : " + str((dz_dq - dz_dq2).max()) )
print( "M : " + str((M - M2).max()) )
print( "I : " + str((I - I2).max()) )
print( "g : " + str(g - g2) )
'''
