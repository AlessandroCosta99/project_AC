import os
import pandas as pd

# Define the root directory path where the subfolders are located
root_dir = '/home/alessandro/Dataset/strawberry_pos_prediction (copy)'

# Function to check if a CSV file contains a value smaller than  50 in the first column
def find_csv_with_small_value(root_dir):
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename == ('strawberry_position.csv'):
                file_path = os.path.join(dirpath, filename)
                try:
                    df = pd.read_csv(file_path)
                    if df.iloc[:,  1].min() <  70:
                        print(f"Found CSV file with value smaller than  50 in the first column: {file_path}")
                        return True
                except Exception as e:
                    print(f"Error reading file {file_path}: {e}")
    return False

# Call the function to search for the CSV file
found = find_csv_with_small_value(root_dir)

if not found:
    print("No CSV file with a value smaller than  50 in the first column was found.")