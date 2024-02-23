import os
import csv

def remove_corresponding_lines(pivot_file, other_files, directory):
    # Full file paths for input and output files
    pivot_path = os.path.join(directory, pivot_file)
    other_paths = [os.path.join(directory, file) for file in other_files]

    # Create a set to store the lines from the pivot file
    pivot_lines = set()

    # Read the pivot file and store its lines in the set
    with open(pivot_path, 'r', newline='') as pivot_file:
        reader = csv.reader(pivot_file)
        for row in reader:
            pivot_lines.add(tuple(row))

    # Remove corresponding lines from other files
    for other_path in other_paths:
        lines_to_keep = []
        with open(other_path, 'r', newline='') as other_file:
            reader = csv.reader(other_file)
            for row in reader:
                if tuple(row) not in pivot_lines:
                    lines_to_keep.append(row)

        # Write the filtered lines back to the same file
        with open(other_path, 'w', newline='') as other_file:
            writer = csv.writer(other_file)
            writer.writerows(lines_to_keep)

# Define the directory containing the files
directory = '/home/alessandro/Dataset/before_filtering_dataset/data_sample_2024-02-23-13-54-54'

# Define the pivot and other files
pivot_file = 'times.csv'
other_files = ['franka_states.csv', 'strawberry_position.csv']

# Remove corresponding lines from other files based on the pivot file
remove_corresponding_lines(pivot_file, other_files, directory)
