import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
class TrajectorySmoothing():
    def __init__(self):
        self.compute_spline([0,2],[0,3])
    def compute_spline(self, x, y):
        spl = CubicSpline(x, y, bc_type = 'clamped', extrapolate=True)
        xnew = np.linspace(0,  x[1], num=1001)
        ynew = spl(xnew)
        for i in range(len(xnew)):
            print(f"Intermediate Point {i}: ({xnew[i]}, {ynew[i]})")
if __name__ == '__main__':
    ciao = TrajectorySmoothing()

