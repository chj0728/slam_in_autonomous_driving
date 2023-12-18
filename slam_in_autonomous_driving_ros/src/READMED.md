- [pre-dependencies](#pre-dependencies)
- [ch2/imu\_integration](#ch2imu_integration)
- [imu\_pre\_node.cpp](#imu_pre_nodecpp)
  - [launch启动](#launch启动)
- [kdtree\_node.cpp](#kdtree_nodecpp)
  - [节点测试](#节点测试)
- [icp\_2d.cpp](#icp_2dcpp)
  - [节点测试](#节点测试-1)
- [gmapping.cc](#gmappingcc)
  - [节点测试](#节点测试-2)
- [occupancy\_map.cpp{@q }](#occupancy_mapcppq-)
  - [节点测试](#节点测试-3)
- [lio包](#lio包)

## pre-dependencies

+ fmt

[将fmt编译成动态库](https://blog.csdn.net/weixin_38213410/article/details/123987676)

在CMakelists.txt中开头的部分加入下列语句：
```cmake
# enable building shared Libraries
set(BUILD_SHARED_LIBS ON)
```

```bash
git clone https://github.com/fmtlib/fmt.git
cd fmt
mkdir build
cd build
cmake ..
make
sudo make install
```

+ Sophus
```bash
git clone https://github.comakem/strasdat/Sophus.git
cd Sophus
#git checkout a621ff
mkdir build 
cd build
cmake ..
make
sudo make install
```
+ Ceres-solver
```bash
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev

git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir build 
cd build
cmake ..
make
sudo make install
```

## ch2/imu_integration
+ ROS功能包**imu_integration**根据[run_imu_integration.cc](../../src/ch3/run_imu_integration.cc)修改而来

+ 订阅imu话题，保存积分结果在[data/state.txt](data/state.txt)
```bash
rosrun imu_integration imu_integration_node
```
+ 通过  [plot_ch3_state.py](scripts/plot_ch3_state.py)
```python
python3 ./plot_ch3_state.py
```

<img src="https://gitee.com/cao-haojie/images/raw/master/images/202308122319942.png" alt="1691849588841" style="zoom: 50%;" />

+ 将输出**state.txt**(time,x,y,z,qx,qy,qz,qw,vx,vy,vz)保存为
**tum.txt**格式(time,x,y,z,qx,qy,qz,qw)
```bash
cut -d ' ' -f -8 state.txt >tum.txt
```
+ 使用[EVO工具](https://github.com/MichaelGrupp/evo)可视化数据
```bash
evo_traj tum tum.txt -p
```

## imu_pre_node.cpp
+ 带有imu静止初始化模块的imu预积分节点
+ imu静止初始化的相关参数在 **src/lio/param/params.yaml**
+ 订阅imu话题“imu/data”

### launch启动
+ roslaunch lio start.launch 


## kdtree_node.cpp
+ 添加K-d树的循环实现

### 节点测试
+ rosrun lio kdtree_node 

## icp_2d.cpp

+ 点对点的icp
+ 发布里程计和tf
+ 未考虑运动畸变
+ 设置迭代10次时，回调处理一次的周期大于激光15Hz周期
+ 设置迭代次数为5
+ 基于Ceres优化求解器

### 节点测试
+ rosrun lio icp_node
+ rosbag play data.bag --topics /scan_multi


## gmapping.cc
+ 基于Gmapping，激光雷达数据单次更新栅格地图
+ 转换一次地图用时约: 0.16～0.18秒

### 节点测试
+ rosrun lio gmapping_node
+ rosrun lio icp_node
+ rosbag play data.bag --topics /scan_multi


##  occupancy_map.cpp{@q }
- [ ] 实现基于子图的地图发布

### 节点测试
+ rosrun lio occupancy_node 
+ rosbag play data.bag --topics /scan_multi

## lio包
include/lio/sensors/：
  imu.hpp
  >+ class IMU
  >+ class IMUSubscriber
  
src/sensors/:
  >imu.cpp

src/nodes/:
  > imu_node.cpp

cmake:
```cmake
file(GLOB_RECURSE ALL_SRCS "src/sensors/*.cpp")
add_executable(imu_node 
              src/nodes/imu_node.cpp
              ${ALL_SRCS})
target_link_libraries(imu_node PRIVATE 
                      glog
                      gflags
                      ${catkin_LIBRARIES})
```

-----
include/lio/
