# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 格式：t x y z qx qy qz qw vx vy vz
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        plt.rcParams['figure.figsize'] = (16.0, 12.0)

        # 轨迹
        plt.subplot(221)
        plt.scatter(path_data[:, 1], path_data[:, 2], s=2)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid()
        plt.title('2D trajectory')

        # 姿态
        plt.subplot(222)
        plt.plot(path_data[:, 0], path_data[:, 4], 'r')
        plt.plot(path_data[:, 0], path_data[:, 5], 'g')
        plt.plot(path_data[:, 0], path_data[:, 6], 'b')
        plt.plot(path_data[:, 0], path_data[:, 7], 'k')
        plt.title('q')
        plt.legend(['qw', 'qx', 'qy', 'qz'])

        # 速度
        plt.subplot(224)
        plt.plot(path_data[:, 0], path_data[:, 8], 'r')
        plt.plot(path_data[:, 0], path_data[:, 9], 'g')
        plt.plot(path_data[:, 0], path_data[:, 10], 'b')
        plt.title('v')
        plt.legend(['vx', 'vy', 'vz'])

        # plt.show()        
        
        
        # creat 3D trajector

        # 方法1
        #plt.figure()
        #ax = plt.axes(projection = '3d')

        # 方法2
        ax = plt.subplot(223, projection='3d')
        # ax = plt.figure().gca(projection='3d')

        ax.scatter3D(path_data[:, 1], path_data[:, 2], path_data[:, 3] ,s=2, c="b")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D trajector')
        plt.legend(['position'],loc="best")#注释

        plt.show()

        exit(1)
