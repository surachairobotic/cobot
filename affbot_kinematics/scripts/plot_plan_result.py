#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
  lines = []
  with open('plan_result.txt','rt') as f:
    line = f.readline()
    while line:
      vals = line.split(' ')
      if len(vals)!=4:
        print('invalid val num : ' + str(len(vals)))
        exit()
      v = []
      for i in range(len(vals)):
        v.append(float(vals[i]))
      lines.append(v)
      line = f.readline()
  plans = []
  with open('plan2.txt','rt') as f:
    line = f.readline()
    while line:
      vals = line.split(' ')
      if len(vals)!=17:
        print('invalid val num : ' + str(len(vals)))
        exit()
      v = []
      for i in range(len(vals)-1):
        v.append(float(vals[i]))
      plans.append(v[0:2])
      line = f.readline()
  lines = np.array(lines)
  plans = np.array(plans)
  t = lines[:,0]
  q = lines[:,1]
  dq = lines[:,2]
  qt = lines[:,3]
  plan_t = plans[:,0]
  plan_q = plans[:,1]
  f, axarr = plt.subplots(2, sharex=True)
  axarr[0].hold(True)
  axarr[1].hold(True)
  axarr[0].plot(t, q, '+-', t, qt, 'g+-', plan_t, plan_q, 'r+-')
  axarr[1].plot(t, dq, '+-')
  axarr[0].set_xlim([0, t[-1]])
  plt.show()
