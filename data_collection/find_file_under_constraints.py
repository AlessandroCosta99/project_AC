
# import os
# import csv

# def find_csv_with_conditions(root_folder):
#     for root, dirs, files in os.walk(root_folder):
#         for file in files:
#             if file == 'strawberry_position.csv':
#                 csv_path = os.path.join(root, file)
#                 with open(csv_path, 'r') as csvfile:
#                     csvreader = csv.reader(csvfile)
#                     next(csvreader)  # Skip the first row
#                     for row in csvreader:
#                         if len(row) >= 2:
#                             if float(row[1]) > 300 or float(row[0]) < 300:
#                                 return csv_path
#     return None

# root_folder = '/home/alessandro/Dataset/strawberry_pos_prediction (copy)'
# result = find_csv_with_conditions(root_folder)
# if result:
#     print(f"CSV file with conditions found at: {result}")
# else:
#     print("No CSV file found matching the conditions.")


import os
import csv

def check_csv_files(folder_path):
    subfolder_paths = set()
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith("strawberry_position.csv"):
                csv_path = os.path.join(root, file)
                with open(csv_path, 'r') as csvfile:
                    csvreader = csv.reader(csvfile)
                    for row in csvreader:
                        try:
                            value_x = float(row[0])
                            value_y = float(row[1])
                            if value_x < 415 and value_y<102:
                                subfolder_paths.add(os.path.dirname(csv_path))
                                break
                        except ValueError:
                            # Skip rows that don't have a valid float in the first column
                            continue
    return subfolder_paths

# Example usage:
parent_folder_path = "/home/alessandro/Dataset/first_collection_100push"
matching_files = check_csv_files(parent_folder_path)
for file_path in matching_files:
    print(file_path)
