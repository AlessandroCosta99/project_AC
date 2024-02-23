
import os
import csv

def find_csv_with_conditions(root_folder):
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file == 'strawberry_position.csv':
                csv_path = os.path.join(root, file)
                with open(csv_path, 'r') as csvfile:
                    csvreader = csv.reader(csvfile)
                    next(csvreader)  # Skip the first row
                    for row in csvreader:
                        if len(row) >= 2:
                            if float(row[1]) > 300 or float(row[0]) < 300:
                                return csv_path
    return None

root_folder = '/home/alessandro/Dataset/strawberry_pos_prediction (copy)'
result = find_csv_with_conditions(root_folder)
if result:
    print(f"CSV file with conditions found at: {result}")
else:
    print("No CSV file found matching the conditions.")
