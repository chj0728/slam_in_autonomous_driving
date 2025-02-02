# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 格式：t x y z qx qy qz qw
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        # fig = plt.figure()
        # ax = Axes3D(fig)
        # ax.plot(path_data[:, 1], path_data[:, 2], path_data[:, 3])
        # plt.title('3D path')
        # plt.show()
        # exit(1)   

        plt.rcParams['figure.figsize'] = (9.0, 6.0)
        plt.rcParams['figure.dpi'] = 200
        plt.rcParams['savefig.dpi'] = 1000
        # plt.rcParams['figure.figsize'] = (12.0, 6.0)
        # plt.rcParams['font.size'] = 10
        # plt.rcParams['legend.fontsize'] = 'small'
        # plt.rcParams['figure.titlesize'] = 'medium' 
        # plt.rcParams['lines.linewidth'] = 0.5
        # plt.rcParams['lines.markersize'] = 0.5 
        # plt.rcParams['lines.color'] = 'b'
        ax = plt.subplot(projection='3d')
        ax.plot3D(path_data[:, 1], path_data[:, 2], path_data[:, 3], 'r',alpha = 0.8)
        ax.set_title('3D path')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.legend(['3Dposition'],loc="best")   
        plt.show()
        exit(1)