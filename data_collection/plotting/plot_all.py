import os
import pandas as pd
import matplotlib.pyplot as plt

# Define the path to the parent directory
parent_dir = '/home/robofruit2/Desktop/fourth_collection'



# Iterate through each subfolder and file
for dirpath, dirnames, filenames in os.walk(parent_dir):
    # Break the loop if we've processed  10 folders
    for filename in filenames:
        # if filename == 'strawberry_position.csv':
        #     # Construct the full file path
        #     file_path = os.path.join(dirpath, filename)
            
        #     # Read the CSV file into a DataFrame
        #     df = pd.read_csv(file_path)

        #     # Plot the data (assuming 'x' and 'y' are the column names)
        #     plt.plot(df['berry_x'].to_numpy(), df['berry_y'].to_numpy())
        #     plt.scatter(df['berry_x'].to_numpy(), df['berry_y'].to_numpy(), color='red', marker='o')
            
        #     # Customize the plot
        #     plt.title(f"Plot of X and Y Coordinates for {os.path.basename(dirpath)}")
        #     plt.xlabel("X Coordinate")
        #     plt.ylabel("Y Coordinate")
            

        if filename == 'franka_states.csv':
            # Construct the full file path
            file_path = os.path.join(dirpath, filename)
            
            # Read the CSV file into a DataFrame
            df = pd.read_csv(file_path)
            
            # Plot the data (assuming 'x' and 'y' are the column names)
            plt.plot(df['O_T_EE13'].to_numpy(), df['O_T_EE14'].to_numpy())
            #plt.scatter(df['berry_x'].to_numpy(), df['berry_y'].to_numpy(), color='red', marker='o')
            
            # Customize the plot
            plt.title(f"Plot of X and Y Coordinates for {os.path.basename(dirpath)}")
            plt.xlabel("X Coordinate")
            plt.ylabel("Y Coordinate")
# Show all the figures
plt.show()
