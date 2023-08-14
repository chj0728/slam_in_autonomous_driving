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