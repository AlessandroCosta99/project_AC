
import numpy as np 
from scipy.interpolate import griddata
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D 
from matplotlib.colors import LinearSegmentedColormap
  
# Generate random 3D data points 
import numpy as np

# Sample data for demonstration
T = np.array([2, 5, 10])  # T values
N = np.array([3, 5, 7, 10])  # N values
comp_times = np.array([
    [0.6, 1.1, 2.8, 5.6],  # Times for T=2
    [7, 13.6, 21.4, 34.6],  # Ensuring monotonicity
    [28.7, 114, 200, 210]  # Ensuring monotonicity
])

# Ensuring monotonicity of z values for each T
# Assuming initial data might not be strictly monotonic and needs adjustment
# This simple approach ensures monotonicity by taking the max of current and all previous values
for i in range(comp_times.shape[0]):
    for j in range(1, comp_times.shape[1]):
        comp_times[i, j] = max(comp_times[i, j], comp_times[i, j-1])

# Flatten the arrays and create an N x 3 array as described
x = np.repeat(T, len(N))  # Repeating each T value for each N
y = np.tile(N, len(T))  # Repeating the sequence of N for each T
z = comp_times.flatten()  # Flatten the comp_times to correspond to the x, y pairs

data = np.array([x, y, z]).T  # Transpose to get N x 3 shape

# Now data is ready and formatted as required.
print("Data (x, y, z):")
print(data)

T = data[:, 0]
N = data[:, 1]
comp_times = data[:, 2]

# Creating a grid for interpolation
T_grid, N_grid = np.mgrid[min(T):max(T):100j, min(N):max(N):100j]

# Interpolating the computational times
comp_times_grid = griddata((T, N), comp_times, (T_grid, N_grid), method='cubic')

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plotting the surface, with NaN values handled by specifying a limit or using 'linear' interpolation as fallback


cmap_name = 'custom_green_yellow_red'
colors = [(0, 'green'), (0.01, 'green'), (0.01, 'yellow'), (1, 'red')]
# cmap = LinearSegmentedColormap.from_list(cmap_name, colors, N=256)
cmap = LinearSegmentedColormap.from_list('custom_green_yellow_red', ['green', 'yellow', 'red'], N=256)

surf = ax.plot_surface(T_grid, N_grid, comp_times_grid, cmap=cmap, edgecolor='none')

ax.set_xlabel('T')
ax.set_ylabel('N')
ax.set_zlabel('Computational Time (s)')
ax.set_title('Interpolated 3D Surface of Computational Times')

# Adding color bar for reference
#fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()