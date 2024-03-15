import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 45, 160)
print(x.shape)

y_random = np.random.normal(0, 0.2, size=x.shape)
window_size = 5
y_smooth = np.convolve(y_random, np.ones(window_size)/window_size, mode='same') 

y_opposite = -y_smooth


x_first_half = x[:50]
y_smooth_first_half = y_smooth[:50]
y_opposite_first_half = y_opposite[:50]

x_second_half = x[10:] - x[10]  # Adjust x values to start from 0 for the second half
y_smooth_second_half = y_smooth[10:]
y_opposite_second_half = y_opposite[10:]
y_smooth_first_half_adjusted = np.copy(y_smooth_first_half)
N = 4 
y_smooth_first_half_adjusted[-N:] = np.linspace(y_smooth_first_half_adjusted[-N], 0, N)


fig, axs = plt.subplots(2, 1, figsize=(10, 2), sharex=False)

for ax in axs:
    # Plot the 0 axes
    ax.axhline(0, color='black', linewidth=1)  # Horizontal line at y=0
    ax.axvline(0, color='black', linewidth=1)  # Vertical line at x=0
    
    # Remove the box around the plots
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_visible(False)
    
    ax.grid(False)
    ax.set_yticklabels([])
    ax.set_aspect(aspect =2 , adjustable='box')

axs[0].set_xticklabels([])

# Plot the first half in the first subplot
axs[0].plot(x_first_half, y_smooth_first_half_adjusted,  color='darkgreen')


# Plot the second half in the second subplot, adjusted to start from 0
axs[1].plot(x_second_half, y_opposite_second_half,  color='magenta')


plt.tight_layout(pad=0.4, h_pad=0.5)
plt.show()