import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots(4,1)
fig2, bx = plt.subplots()

for x in ax:
  x.axis([0, 100, 0, 1])
bx.axis([0, 100, 0, 1])

y = [0]*4
lines = [0]*4
for i in range(4):
  y[i] = np.random.rand(100)
  lines[i] = ax[i].plot(y[i])
line_b = [0]*3
line_b[0] = bx.plot(y[0])
line_b[1] = bx.plot(y[1])
line_b[2] = bx.plot(y[2])

fig.canvas.manager.show() 
fig2.canvas.manager.show() 

while 1:
  for i in range(4):
    y[i] = np.random.rand(100)
    lines[i][0].set_ydata(y[i])
  line_b[0][0].set_ydata(y[0])
  line_b[1][0].set_ydata(y[1])
  line_b[2][0].set_ydata(y[2])

#  print(y[0].shape)
  fig.canvas.draw()
  fig.canvas.flush_events()
  fig2.canvas.draw()
  fig2.canvas.flush_events()
