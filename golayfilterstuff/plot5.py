# display data from data.txt in matplotlib

import math, time
import numpy as np
import matplotlib.pyplot as plt

def fixdata(data, window_size=21, order=5):
  a = SavitzkyGolayFilter(data, window_size=window_size, order=order)

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

  fixeddata = []
  for j in range(len(a)):
    fixeddata.append(a[j] / max(a))
  localMaxOut = localMaxList(fixeddata)
  localMinOut = localMinList(fixeddata)

  if len(localMaxOut) != 2 or len(localMinOut) != 1:
    return None
  return [localMaxOut[0], localMaxOut[1], localMinOut[0]]

def SavitzkyGolayFilter(data, window_size=21, order=5):
  data = np.array(data)
  deriv=0
  rate=1
  order_range = range(order + 1)
  half_window = (window_size - 1) // 2
  b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
  m = np.linalg.pinv(b).A[deriv] * rate**deriv * math.factorial(deriv)
  firstvals = data[0] - np.abs(data[1:half_window+1][::-1] - data[0])
  lastvals = data[-1] + np.abs(data[-half_window-1:-1][::-1] - data[-1])
  data = np.concatenate((firstvals, data, lastvals))
  return np.convolve(m[::-1], data, mode='valid').tolist()

def localMaxList(data):
  order=1

  locs = list(range(0, len(data)))

  results = [True] * len(data)
  main = take(data, locs)
  for shift in range(1, order + 1):
    plus = take(data, [i + shift for i in locs])
    minus = take(data, [i - shift for i in locs])
    results = andequal(results, [main[i] > plus[i] for i in range(len(main))])
    results = andequal(results, [main[i] > minus[i] for i in range(len(main))])
  return [i for i, val in enumerate(results) if val]

def localMinList(data):
  order=1

  locs = list(range(0, len(data)))

  results = [True] * len(data)
  main = take(data, locs)
  for shift in range(1, order + 1):
    plus = take(data, [i + shift for i in locs])
    minus = take(data, [i - shift for i in locs])
    results = andequal(results, [main[i] < plus[i] for i in range(len(main))])
    results = andequal(results, [main[i] < minus[i] for i in range(len(main))])
  return [i for i, val in enumerate(results) if val]

def take(array, indices):
  out = []
  for i in indices:
    if i < len(array):
      out.append(array[i])
    else:
      out.append(array[len(array) - 1])
  return out

def andequal(list1, list2):
  out = []
  for i in range(len(list1)):
    if list1[i] == list2[i]:
      out.append(list1[i])
    else:
      out.append(False)
  return out

def getdriveoverangle(data):
  allminmax = []
  for i in range(len(data)):
    breakOn = False
    for j in range(7, 15, 2):
      if breakOn:
        break
      for k in range(21, 31, 2):
        if breakOn:
          break
        a = fixdata(data[i], k, j)
        if a != None:
          breakOn = True
          allminmax.append(a)
          break
  out = [allminmax[0][i] - allminmax[1][i] for i in range(len(allminmax[0]))]
  return sum(out) / len(out)

# read data from file
data = np.loadtxt('data.txt', delimiter=' ')
data = [list(data[:, 0]), list(data[:, 1])]
print(getdriveoverangle(data))