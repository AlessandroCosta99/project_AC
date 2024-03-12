import numpy as np
import matplotlib.pyplot as plt

# Define the start and end points
start = [0.6, -0.09]
end = [0.5, -0.23]
end_2 = [0.7, -0.25]
end_3 = [0.58, -0.29]

# Generate the straight line
x = np.linspace(start[0], end[0], 50)
y = np.linspace(start[1], end[1], 50)

x_2 = np.linspace(start[0], end_2[0], 50)
y_2 = np.linspace(start[1], end_2[1], 50)

x_3 = np.linspace(start[0], end_3[0], 50)
y_3 = np.linspace(start[1], end_3[1], 50)

noise1 = np.random.normal(0, 0.0015, size=x.shape)
noisy_x = x + noise1
noisy_x[0], noisy_x[-1] = x[0], x[-1]
noise2 = np.random.normal(0, 0.0015, size=x.shape)
noisy_x2 = x_2 + noise2
noisy_x2[0], noisy_x2[-1] = x_2[0], x_2[-1]
noise3 = np.random.normal(0, 0.0015, size=x.shape)
noisy_x3 = x_3 + noise3
noisy_x3[0], noisy_x3[-1] = x_3[0], x_3[-1]

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(y, x, color = 'grey', label='Reference trajectory', linestyle='--')
plt.plot(y, noisy_x, color = 'blue', label='Executed trajectory')
plt.scatter( start[1], start[0], color='red')
plt.scatter( end[1], end[0], color='green')
plt.text(end[1], end[0], ' Target', horizontalalignment='left', verticalalignment='bottom', color='green')
###########################
plt.plot(y_2, x_2, color = 'grey', label='Reference trajectory', linestyle='--')
plt.plot(y_2, noisy_x2, color = 'blue', label='Executed trajectory')
plt.scatter( start[1], start[0], color='red')
plt.scatter( end_2[1], end_2[0], color='green')
plt.text(end_2[1], end_2[0], ' Target', horizontalalignment='left', verticalalignment='top', color='green')
###########################
plt.plot(y_3, x_3, color = 'grey', label='Reference trajectory', linestyle='--')
plt.plot(y_3, noisy_x3, color = 'blue', label='Executed trajectory')
plt.scatter( start[1], start[0], color='red')
plt.scatter( end_3[1], end_3[0], color='green')
plt.text(end_3[1], end_3[0], ' Target', horizontalalignment='left', verticalalignment='top', color='green')


plt.text(start[1], start[0], ' Start', horizontalalignment='left', verticalalignment='bottom', color='red')
plt.xlabel('Y')
plt.ylabel('X')
plt.title('Example of an executed trajectory in robot base frame')
plt.axis('equal')
plt.legend(['reference trajectory', 'executed trajectory'])
plt.show()

