import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

# Original data
T = np.array([2, 5, 10])  # T values
N = np.array([3, 5, 7, 10])  # N values
comp_times = np.array([
    [0.6, 1.1, 2.8, 5.6],   # Times for T=2
    [7, 13.6, 21.4, 34.6],   # Adjusted to ensure monotonicity
    [28.7, 114, 200, 310]  # Adjusted to ensure monotonicity
])

# Flatten the arrays for griddata input
T_flattened, N_flattened = np.meshgrid(T, N)
T_flattened = T_flattened.flatten()
N_flattened = N_flattened.flatten()
comp_times_flattened = comp_times.flatten()

# Create a grid on which to interpolate
T_grid, N_grid = np.mgrid[min(T):max(T):100j, min(N):max(N):100j]

# Interpolate using griddata
comp_times_grid = griddata((T_flattened, N_flattened), comp_times_flattened, (T_grid, N_grid), method='linear')

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plotting the surface
surf = ax.plot_surface(T_grid, N_grid, comp_times_grid, cmap='viridis', edgecolor='none')

ax.set_xlabel('T')
ax.set_ylabel('N')
ax.set_zlabel('Computational Time (s)')
ax.set_zlim(0, 50)  # Limiting the Z-axis (Computational Time axis) to 200

fig.colorbar(surf, shrink=0.5, aspect=5)  # Add a color bar which maps values to colors.

plt.show()
