import numpy as np
import matplotlib.pyplot as plt

# Replace 'path_to_file.npy' with the actual file path
file_path = '/home/alessandro/Desktop/project_AC/results_data/001/localisation.npy'

# Load the .npy file
data = np.load(file_path, allow_pickle=True)
print(data)
# Now 'data' contains the numpy array stored in the .npy file
plt.figure(figsize=(10, 6))  # Set the figure size (optional)
plt.plot(data)  # Plot the data
plt.title('Data Plot')  # Set the title of the plot
plt.xlabel('Index')  # Label for the x-axis
plt.ylabel('Value')  # Label for the y-axis
plt.grid(True)  # Show grid lines (optional)
plt.show()  # Display the plot