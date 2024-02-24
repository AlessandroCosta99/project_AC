import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
df = pd.read_csv("/home/alessandro/Dataset/first_collection_100push/data_sample_2024-02-24-13-10-03/strawberry_position.csv")

# Extract the columns as NumPy arrays
column1 = df.iloc[:, 0].values  # Assuming the first column is x-axis data
column2 = df.iloc[:, 1].values  # Assuming the second column is y-axis data

# Plot the data
plt.plot(column1, column2)
plt.xlabel('X-axis Label')  # Replace 'X-axis Label' with an appropriate label
plt.ylabel('Y-axis Label')  # Replace 'Y-axis Label' with an appropriate label
plt.title('Your Title')     # Replace 'Your Title' with an appropriate title
plt.grid(True)              # Add grid if needed
plt.show()
