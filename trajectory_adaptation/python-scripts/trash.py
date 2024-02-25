import numpy as np

# Define the 3x3 matrix
matrix_3x3 = np.array([[1, 2, 3],
                       [4, 5, 6]])

# Subtract the lines to get a 2x2 matrix
result_matrix_2x2 = np.array([[matrix_3x3[1, :] - matrix_3x3[0, :]]])

print("Resultant 2x2 Matrix:")
print(result_matrix_2x2)
