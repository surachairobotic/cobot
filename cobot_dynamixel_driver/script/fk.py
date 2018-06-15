
from math import pi, cos, sin
import numpy as np
import copy
H_PI = pi/2.0;
L = [0.184, 0.27203, 0.141, 0.109, 0.111, 0.05]
a = [0, 0, L[1], 0, 0, 0, 0]
alpha = [0, -H_PI, 0, -H_PI, H_PI, -H_PI, 0]
d = [L[0], 0, 0, L[2]+L[3], 0, L[4], L[5]]

T = []
for i in range(7):
  x = np.zeros([4,4])
  x[3,3] = 1.0
  T.append(x)

def fk(theta):
  global T, alpha, d
  dh_theta = [theta[0], theta[1]-H_PI, theta[2], theta[3], theta[4]+H_PI, theta[5], 0];
  for i in range(len(dh_theta)):
    cd = cos(dh_theta[i])
    ca = cos(alpha[i])
    sd = sin(dh_theta[i])
    sa = sin(alpha[i])
    x = T[i]
    x[0,0] = cd
    x[0,1] = -sd
    x[0,2] = 0.0
    x[0,3] = a[i]
    x[1,0] = sd*ca
    x[1,1] = cd*ca
    x[1,2] = -sa
    x[1,3] = -sa*d[i]
    x[2,0] = sd*sa
    x[2,1] = cd*sa
    x[2,2] = ca
    x[2,3] = ca*d[i]
  t_all = copy.deepcopy(T[0])
  for i in range(6):
    t_all = np.matmul( t_all, T[i+1] )
  return t_all[0:3, 3]  

if __name__ == "__main__":
  print(fk([0,0,0,0,0,0]))
  


