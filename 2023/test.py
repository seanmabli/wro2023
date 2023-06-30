import matplotlib.pyplot as plt

data = [0.86015211, 0.66480249, 0.19880487, 0.42112983, 0.01780584,
        0.62128139, 0.47819413, 0.68326718, 0.53730458, 0.89080319,
        0.52955107, 0.60594081]

plt.plot(data)
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Plot of the given sequence')
plt.show()
