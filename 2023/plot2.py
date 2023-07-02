# display data from data.txt in matplotlib

from math import factorial
import numpy as np
from scipy.signal import argrelextrema

def fixdata(data, window_size=21, order=5):
  # prep data
  out = []
  start = []
  end = []
  for i in range(len(data[0, :])):
    deriv=0
    rate=1
    y = data[:,i].copy()
    print(y)
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    firstvals = y[0] - np.abs(y[1:half_window+1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    a = np.convolve(m[::-1], y, mode='valid')

    for j in range(len(a) - 1):
      if a[j + 1] - a[j] > 3:
        f = [a[j]] * len(a[:j])
        g = a[j:]
        f.extend(g)
        a = f.copy()
        break

    for j in reversed(range(len(a) - 1)):
      if a[j] - a[j + 1] > 3:
        f = a[:j + 1]
        g = [a[j + 1]] * len(a[j + 1:])
        f.extend(g)
        a = f.copy()
        break

    out.append(a)


  # plot data
  maxandmin = []
  for i in range(len(out)):
    x = np.arange(0, len(out[i]), 1)

    fixeddata = out[i] / np.max(out[i])
    try:
      maxandmin.append([argrelextrema(fixeddata, np.greater)[0][0], argrelextrema(fixeddata, np.greater)[0][1], argrelextrema(fixeddata, np.less)[0][0]])
    except:
      return None

  # get differnce
  return sum([a_i - b_i for a_i, b_i in zip(maxandmin[0], maxandmin[1])]) / len(maxandmin[0])

def getdriveoverangle(data):
  a = fixdata(data)
  b = 7
  while a == None:
    a = fixdata(data, 21, b)
    b += 2
  return a

# read data from file
data = np.loadtxt('data.txt', delimiter=' ')
print(getdriveoverangle(data))