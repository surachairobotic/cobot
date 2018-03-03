#!/usr/bin/env python
import math

dt = 0.1
t = 0

with open('plan2.txt', 'wt') as f:
  for i in range(50):
    q = 3*math.sin(math.pi*t/4)
    s = str(t)+' '+str(q)+' '
    for i in range(14):
      s+= '0 '
    f.write(s+'\n')
    t+= dt
