import pickle, math
import numpy as np

data = {}
for order in range(7, 15, 2):
  for window_size in range(21, 31, 2):
    deriv=0
    rate=1
    half_window = (window_size - 1) // 2
    b = np.mat([[k**i for i in range(order + 1)] for k in range(-half_window, half_window+1)])
    data[f"{window_size},{order}"] = (np.linalg.pinv(b).A[deriv] * rate**deriv * math.factorial(deriv)).tolist()[::-1]

with open("data.pickle", "wb") as f:
  pickle.dump(data, f)