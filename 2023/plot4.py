import math, time
import numpy as np
import matplotlib.pyplot as plt

def fixdata(data, window_size=21, order=5):
  # prep data
  out = []
  start = []
  end = []
  for i in range(len(data)):
    newdata = SavitzkyGolayFilter(data[i], window_size=window_size, order=order)

    for j in range(len(newdata) - 1):
      if newdata[j + 1] - newdata[j] > 3:
        f = [newdata[j]] * len(newdata[:j])
        g = newdata[j:]
        f.extend(g)
        newdata = f.copy()
        break

    for j in reversed(range(len(newdata) - 1)):
      if newdata[j] - newdata[j + 1] > 3:
        f = newdata[:j + 1]
        g = [newdata[j + 1]] * len(newdata[j + 1:])
        f.extend(g)
        newdata = f.copy()
        break

    out.append(newdata)

  maxandmin = []
  for i in range(len(out)):
    x = list(range(0, len(out[i])))

    for j in range(len(out[i])):
      out[i][j] = out[i][j] / max(out[i])
    print(localMaxList(out[i]), localMinList(out[i]))
    try:
      maxandmin.append([localMaxList(fixeddata)[0], localMaxList(fixeddata)[1], localMinList(fixeddata)[0]])
    except:
      return None

  # get differnce
  return sum([a_i - b_i for a_i, b_i in zip(maxandmin[0], maxandmin[1])]) / len(maxandmin[0])

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
  a = fixdata(data)
  b = 7
  print(a)
  while a == None:
    a = fixdata(data, 21, b)
    b += 2
  return a

# read data from file
data = np.loadtxt('data.txt', delimiter=' ')
data = [list(data[:, 0]), list(data[:, 1])]
getdriveoverangle(data)