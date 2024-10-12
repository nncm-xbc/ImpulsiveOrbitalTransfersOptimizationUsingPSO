import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("build/convergence_log.csv", delimiter=",")
iter = data[:, 0]
values = data[:, 1]
inertiaWeight = data[:, 2]
socialWeight = data[:, 3]
cognitiveWeight = data[:, 4]

plt.figure(figsize=(12, 6))
plt.plot(iter, values, "r-", label="Calculated Values")
# plt.plot(iter, inertiaWeight, "g-", label="inertiaWeight")
# plt.plot(iter, socialWeight, "b-", label="socialWeight")
# plt.plot(iter, cognitiveWeight, "m-", label="cognitiveWeight")

plt.xlabel("iterations")
plt.ylabel("Values")
plt.legend()
plt.grid(True)

plt.savefig("conv_plot.png")
