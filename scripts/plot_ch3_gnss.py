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
        plt.rcParams['figure.figsize'] = (12.0, 6.0)

        # 2D轨迹
        plt.subplot(121)
        plt.scatter(path_data[:, 1], path_data[:, 2], s=2)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid()
        plt.title('2D trajectory')


        ax = plt.subplot(122, projection='3d')

        ax.plot3D(path_data[:, 1], path_data[:, 2], path_data[:, 3], 'r',alpha = 0.8)
        ax.set_title('3D path')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.show()
        exit(1)
