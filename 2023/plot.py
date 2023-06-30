# display data from data.txt in matplotlib

from math import factorial
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import argrelextrema

def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    try:
        window_size = np.abs(int(window_size))
        order = np.abs(int(order))
    except:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')

# read data from file
data = np.loadtxt('data.txt', delimiter=' ')

# prep data
out = []
start = []
end = []
for i in range(len(data[0, :])):
  # plt.plot(x, data[:,i])
  a = savitzky_golay(data[:,i], 21, 5)

  for j in range(len(a) - 1):
    if a[j + 1] - a[j] > 3:
      start.append(j)
      break

  for j in reversed(range(len(a) - 1)):
    if a[j] - a[j + 1] > 3:
      end.append(j)
      break

  out.append(a)

for i in range(len(out)):
  x = np.arange(0, len(out[i]), 1)
  plt.plot(x, out[i])

for i in range(len(out)):
  a = [out[i][start[i]]] * len(out[i][:start[i]])
  b = out[i][start[i]:end[i]]
  c = [out[i][end[i]]] * len(out[i][end[i]:])
  a.extend(b)
  a.extend(c)
  out[i] = np.array(a)

# plot data
maxandmin = []
for i in range(len(out)):
  x = np.arange(0, len(out[i]), 1)
  plt.plot(x, out[i])

  fixeddata = out[i] / np.max(out[i])
  maxandmin.append([argrelextrema(fixeddata, np.greater)[0][0], argrelextrema(fixeddata, np.greater)[0][1], argrelextrema(fixeddata, np.less)[0][0]])
  for y in list(argrelextrema(fixeddata, np.greater)):
    plt.plot(y, out[i][y], 'ro')
  for y in list(argrelextrema(fixeddata, np.less)):
    plt.plot(y, out[i][y], 'bo')
    
# get differnce
diff = sum([a_i - b_i for a_i, b_i in zip(maxandmin[0], maxandmin[1])]) / len(maxandmin[0])
print(diff)

plt.show()
