import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray
from collections import deque

class TrajectorySmoothing():
    def __init__(self):
        self.next_pose_sub = rospy.Subscriber('/next_pose', Float64MultiArray, self.target_pose_callback)
        self.traj_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=1000)
        self.x_init = 0.0
        self.y_init = 0.0
        self.num_of_discretization = 10001   #changable
        self.x_vec = deque(maxlen=2)
        self.y_vec = deque(maxlen=2)
        self.first_it = True

    def compute_spline(self, x_vec, y_vec):  #x and y are vector of two elements
        #xvec and y vec are received correctly
        spl = CubicSpline(x_vec, y_vec, bc_type = 'clamped')
        xnew = np.linspace(x_vec[0],  x_vec[1], num=self.num_of_discretization)
        ynew = spl(xnew)
        next_coordinate = next_coordinate = np.column_stack((xnew, ynew))
        print(next_coordinate)
        traj_msg = Float64MultiArray(data=next_coordinate)
        self.traj_pub.publish(traj_msg)
        
        
    def plot_trajectory(x,y,spl):   #will not work
        fig, ax = plt.subplots(4, 1, figsize=(5, 7))
        xnew = np.linspace(0, 10, num=10001)
        ax[0].plot(xnew, spl(xnew))
        ax[0].plot(x, y, 'o', label='data')
        ax[1].plot(xnew, spl(xnew, nu=1), '--', label='1st derivative')
        ax[2].plot(xnew, spl(xnew, nu=2), '--', label='2nd derivative')
        ax[3].plot(xnew, spl(xnew, nu=3), '--', label='3rd derivative')
        for j in range(4):
            ax[j].legend(loc='best')
        plt.tight_layout()
        plt.show()

    def target_pose_callback(self, msg):
        x = msg.data[0]
        y = msg.data[1]
        self.x_vec.append(x)
        self.y_vec.append(y)
        if len(self.x_vec) >=  2:
            self.compute_spline(list(self.x_vec), list(self.y_vec))
 


if __name__ == '__main__':
    rospy.init_node("trajectory_smoothing", anonymous=True, disable_signals=True)
    ciao = TrajectorySmoothing()
    rospy.spin()
    
