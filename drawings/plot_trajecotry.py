import numpy as np
import matplotlib.pyplot as plt

# Define the start and end points
start = [0.6, -0.09]
end = [0.5, -0.27]

# Generate the straight line
x = np.linspace(start[0], end[0], 50)
y = np.linspace(start[1], end[1], 50)

# x = np.linspace(start[0], end[0], 100)
# y = np.linspace(start[1], end[1], 100)

# Generate the slightly randomly perturbed line around the straight line
noise = np.random.normal(0, 0.002, size=x.shape)
noisy_x = x + noise
# Ensure start and end points are the same for both lines
noisy_x[0], noisy_x[-1] = x[0], x[-1]

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(y, x, label='Reference trajectory', linestyle='--')
plt.plot(y, noisy_x, label='Executed trajectory')
plt.scatter( start[1], start[0], color='red')
plt.text(start[1], start[0], ' Start', horizontalalignment='right', verticalalignment='bottom', color='red')
plt.scatter( end[1], end[0], color='green')
plt.text(end[1], end[0], ' Target', horizontalalignment='left', verticalalignment='top', color='green')
plt.legend(loc = 'upper left')
plt.xlabel('Y')
plt.ylabel('X')
plt.title('Example of an executed trajectory in robot base frame')
plt.axis('equal')
plt.show()
