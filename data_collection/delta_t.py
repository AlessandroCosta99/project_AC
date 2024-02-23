import os
import pandas as pd

# Specify the directory you want to start from
root_dir = '/home/alessandro/Dataset/new_dataset_times (copy)'
tutte = []
# Use os.walk to iterate through the directory and subdirectories
for dirpath, dirnames, filenames in os.walk(root_dir):
    # Check each file in the current directory
    for filename in filenames:
        # If the file is named 'times.csv', read it using pandas
        if filename == 'times.csv':
            file_path = os.path.join(dirpath, filename)
            df = pd.read_csv(file_path)
            
            # Assuming the column of floats is named 'float_column'
            # Replace 'float_column' with the actual column name
            float_column = df['t']
            
            # Compute the difference between consecutive values
            differences = float_column.diff().dropna()
            
            # Filter out differences that are zero
            non_zero_differences = differences[differences !=  0]
            
            # Compute the average of the non-zero differences
            average_difference = non_zero_differences.mean()

            tutte.append(average_difference)
            
            # Print the average difference
x = pd.Series(tutte).mean()
print(x)
