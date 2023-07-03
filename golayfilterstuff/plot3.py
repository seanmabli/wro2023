# display data from data.txt in matplotlib

import math, time
import numpy as np
import matplotlib.pyplot as plt

def fixdata(data, window_size=21, order=5):
  deriv=0
  rate=1
  half_window = (window_size - 1) // 2
  b = np.mat([[k**i for i in range(order + 1)] for k in range(-half_window, half_window+1)])
  m = np.linalg.pinv(b).A[deriv] * rate**deriv * math.factorial(deriv)
  firstvals = data[0] - np.abs(data[1:half_window+1][::-1] - data[0])
  lastvals = data[-1] + np.abs(data[-half_window-1:-1][::-1] - data[-1])
  data = np.concatenate((firstvals, data, lastvals))
  a = np.convolve(m[::-1], data, mode='valid')

  for j in range(len(a) - 1):
    if a[j + 1] - a[j] > 4:
      f = [a[j]] * len(a[:j])
      g = a[j:]
      f.extend(g)
      a = f.copy()
      break

  for j in reversed(range(len(a) - 1)):
    if a[j] - a[j + 1] > 4:
      f = a[:j + 1]
      g = [a[j + 1]] * len(a[j + 1:])
      f.extend(g)
      a = f.copy()
      break

  fixeddata = a / max(a)
  localMaxOut = localMax(fixeddata)
  localMinOut = localMin(fixeddata)

  if len(localMaxOut) != 2 or len(localMinOut) != 1:
    return None
  return [localMax(fixeddata)[0], localMax(fixeddata)[1], localMin(fixeddata)[0]]

def localMax(data):
  axis=0
  order=1
  mode='clip'

  datalen = data.shape[axis]
  locs = np.arange(0, datalen)

  results = np.ones(data.shape, dtype=bool)
  main = data.take(locs, axis=axis, mode=mode)
  for shift in range(1, order + 1):
    plus = data.take(locs + shift, axis=axis, mode=mode)
    minus = data.take(locs - shift, axis=axis, mode=mode)
    results &= main > plus
    results &= main > minus
    if ~results.any():
      return results
  return np.nonzero(results)[0]

def localMin(data):
  axis=0
  order=1
  mode='clip'

  datalen = data.shape[axis]
  locs = np.arange(0, datalen)

  results = np.ones(data.shape, dtype=bool)
  main = data.take(locs, axis=axis, mode=mode)
  for shift in range(1, order + 1):
    plus = data.take(locs + shift, axis=axis, mode=mode)
    minus = data.take(locs - shift, axis=axis, mode=mode)
    results &= main < plus
    results &= main < minus
    if ~results.any():
      return results
  return np.nonzero(results)[0]

def getdriveoverangle(data):
  allminmax = []
  for i in range(len(data[0])):
    breakOn = False
    for j in range(7, 15, 2):
      if breakOn:
        break
      for k in range(21, 31, 2):
        if breakOn:
          break
        a = fixdata(data[:, i], k, j)
        if a != None:
          breakOn = True
          allminmax.append(a)
          break
  out = [allminmax[0][i] - allminmax[1][i] for i in range(len(allminmax[0]))]
  return sum(out) / len(out)

# read data from file
data = np.loadtxt('data.txt', delimiter=' ')
print(getdriveoverangle(data))