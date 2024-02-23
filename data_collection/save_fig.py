import os
import pandas as pd
import matplotlib.pyplot as plt

# Define the root folder where subfolders are located
root_folder = '/home/alessandro/Dataset/new_dataset_times (copy)'

# Walk through each subfolder
for subdir, dirs, files in os.walk(root_folder):
    for file in files:
        # Check if the file is "_strawberry position.csv"
        if file == "_strawberry position.csv":
            # Construct the full file path
            file_path = os.path.join(subdir, file)
            # Read the CSV file into a DataFrame
            df = pd.read_csv(file_path)
            # Assuming the first column is x and the second is y
            x = df.iloc[:,   0]
            y = df.iloc[:,   1]
            
            # Plot the data
            plt.figure(figsize=(10,   6))
            plt.plot(x, y, 'o-')
            plt.xlabel('X-axis label')
            plt.ylabel('Y-axis label')
            plt.title('Strawberry Position')
            
            # Save the figure inside the subfolder
            figure_path = os.path.join(subdir, 'strawberry_position_plot.png')
            plt.savefig(figure_path)
            plt.close()  # Close the plot to free up memory
