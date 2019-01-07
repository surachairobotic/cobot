import numpy as np
from scipy import signal

def data2spectrum(data, t):
  ps = np.abs(np.fft.fft(data))**2
  freq = np.fft.fftfreq(len(data), d=(t[1]-t[0]))
  idx = np.argsort(freq)
  print("freq=%d, data=%d, idx=%d" % (len(freq), len(data), len(idx)))
  fq1 = []
  fq2 = []
  for i in idx:
    fq1.append(freq[i])
    if freq[i] > -0.4 and freq[i] < 0.4:
      ps[i] = 0.0
    fq2.append(ps[i])
  return fq1, fq2

