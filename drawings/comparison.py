import matplotlib.pyplot as plt
import numpy as np
l=160
x = np.linspace(0, 40, l)
y_random = np.random.normal(0, 0.08, size=x.shape)
window_size = 20
y_smooth = np.convolve(y_random, np.ones(window_size)/window_size, mode='same')
y_opposite = -y_smooth

y_random2 = np.random.normal(0, 0.08, size=x.shape)
y_smooth2 = np.convolve(y_random2, np.ones(window_size)/window_size, mode='same')
y_opposite2 = -y_smooth2

x_first_half = x[:l-20]
y_smooth_first_half = y_smooth[:l-20]
y_opposite_first_half = y_opposite[:l-20]

x_second_half = x[20:] - x[20]  # Adjust x values to start from 0 for the second half
y_smooth_second_half = y_smooth2[20:]
y_opposite_second_half = y_opposite2[20:]

y_smooth_first_half_adjusted = np.copy(y_smooth_first_half)
N = 7
y_smooth_first_half_adjusted[-N:] = np.linspace(y_smooth_first_half_adjusted[-N], 0, N)


# Create second figure
# plt.figure(figsize=(10, 3))
# ax = plt.gca()
# ax.axhline(0, color='black', linewidth=1)  # Horizontal line at y=0
# # ax.axvline(0, color='black', linewidth=1)  # Vertical line at x=0
# ax.spines['top'].set_visible(True)
# ax.spines['right'].set_visible(True)
# ax.spines['bottom'].set_visible(True)
# ax.spines['left'].set_position('zero')
# ax.grid(False)
# plt.ylim(-1,1)
# # ax.set_yticklabels([])
# # ax.set_aspect(aspect=2, adjustable='box')
# plt.plot(x_second_half, y_opposite_second_half, color='magenta')
# plt.show()  # Show the second figure

# # Create first figure
# plt.figure(figsize=(10, 3))
# ax = plt.gca()
# ax.axhline(0, color='black',linestyle = '--', linewidth=0.5)  # Horizontal line at y=0
# # ax.axvline(0, color='black', linewidth=1)  # Vertical line at x=0
# ax.spines['top'].set_visible(True)
# ax.spines['right'].set_visible(True)
# ax.spines['bottom'].set_visible(True)
# ax.spines['left'].set_position('zero')
# ax.grid(False)
# plt.ylim(-0.2,0.2)
# plt.plot(x_first_half, y_smooth_first_half_adjusted, color='darkgreen')


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 3))

# Configuration for the first subplot
ax1.axhline(0, color='black', linestyle='--', linewidth=0.5)  # Horizontal line at y=0
ax1.spines['top'].set_visible(True)
ax1.spines['right'].set_visible(True)
ax1.spines['bottom'].set_visible(True)
ax1.spines['left'].set_position('zero')
ax1.grid(False)
ax1.set_ylim(-0.2, 0.2)
ax1.plot(x_first_half, y_smooth_first_half_adjusted, color='darkgreen')
ax1.set_title('Adapted trajectory')

# Configuration for the second subplot
ax2.axhline(0, color='black', linestyle = '--', linewidth=0.5)  # Horizontal line at y=0
ax2.spines['top'].set_visible(True)
ax2.spines['right'].set_visible(True)
ax2.spines['bottom'].set_visible(True)
ax2.spines['left'].set_position('zero')
ax2.grid(False)
ax2.set_ylim(-1, 1)
ax2.plot(x_second_half, y_opposite_second_half, color='magenta')
ax2.set_title('Stem positon normalized')

plt.show()