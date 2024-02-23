import os
import pandas as pd

# Define the root directory path where the subfolders are located
root_dir = '/home/alessandro/Dataset/new_dataset_times (copy)'

# Define a function to remove rows from a DataFrame based on the indices
def remove_rows(df, indices):
    return df[~df.index.isin(indices)]

# Loop through each subdirectory and process the CSV files
for dirpath, dirnames, filenames in os.walk(root_dir):
    # Check if the 'strawberry_position.csv' file exists in the directory
    if 'strawberry_position.csv' in filenames:
        # Construct the full path to the 'strawberry_position.csv' file
        strawberry_file_path = os.path.join(dirpath, 'strawberry_position.csv')
        # Read the 'strawberry_position.csv' file and get the indices where the second column is greater than  380
        strawberry_df = pd.read_csv(strawberry_file_path)
        indices_to_remove = strawberry_df[strawberry_df.iloc[:,  0] <  250].index
        
        # Define the list of CSV file names to process
        csv_files = ['franka_states.csv', 'robot_state.csv', 'strawberry_position.csv', 'times.csv']
        
        # Loop through each CSV file in the directory and remove the rows
        for file in csv_files:
            # Construct the full path to the CSV file
            csv_file_path = os.path.join(dirpath, file)
            # Check if the file exists
            if os.path.isfile(csv_file_path):
                # Read the CSV file into a DataFrame
                df = pd.read_csv(csv_file_path)
                # Remove the rows based on the condition
                df = remove_rows(df, indices_to_remove)
                # Write the modified DataFrame back to the file
                df.to_csv(csv_file_path, index=False)
