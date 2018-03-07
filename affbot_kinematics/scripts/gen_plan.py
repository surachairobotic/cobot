#!/usr/bin/env python
import math

dt = 0.05
t = 0

with open('plan2.txt', 'wt') as f:
  for i in range(400):
    amp = 3
    ft = math.pi/2
    q = amp*math.sin(t*ft)
    dq = amp*math.cos(t*ft)*ft

    v = [0 for i in range(16)]
    v[0] = t
    v[1] = q
    v[6] = dq
    s = ''
    for i in range(16):
      s+= '%.3f ' % (v[i])
    f.write(s+'\n')
    t+= dt
