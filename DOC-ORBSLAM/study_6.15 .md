# 感知 SLAM相关
# 三种slam框架 ：A-LOAM、FAST_LIO、ORB_SLAM

```
共同点：
① 本质上都是在估计： x,y,z; roll,pitch,yaw     =>   机器人在空间中的6自由度位姿
② 都依赖“相邻帧之间的匹配”，都需要判断：当前这一帧传感器数据和上一帧/地图之间有什么关系
  区别只是数据类型不同，都是通过“传感器观测之间的对应关系”来估计运动
③ 都需要用到坐标变换：
  世界坐标系/map 里程计坐标系/odom 机器人坐标系/base_link 传感器坐标系/camera_link /lidar_link imu_link
④ 都会有前端和后端：       整体思想：提取约束-->构建误差-->优化位姿
  前端：从传感器数据中提取约束
  后端：优化位姿，使误差最小
  ORB-SLAM------前端：ORB特征提取与匹配
                后端：Bundle Adjustment/图优化
  A-LOAM--------前端：边缘点、平面点提取
                后端：点到线、点到面残差优化
  FAST-LIO------前端：点云预处理、imu预积分/预测
                后端：ESKF状态更新、点云地图匹配
```



## A-LOAM中遇到的问题与解析：

```
1.插值旋转、插值平移：
插值 --> 估计“雷达在这个中间时刻的姿态”

slerp: Spherical Linear Interpolation  球面线性插值    -----专门用于四元数旋转插值
Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
解释：从“没有旋转”慢慢过渡到“结束时刻的总旋转”，取中间比例为s的那个旋转

坐标系：
curr：当前帧的点云坐标系
last: 上一帧参考坐标系，可理解成“要补偿到的参考时刻”
point: 正在处理的激光点，某个中间时刻ti
T_last_curr: 把curr坐标系下的点转换到last坐标系下。

2.前端二：laserOdometry.cpp   -----前端里程计模块
...当前点运动补偿--->角点匹配--->平面点匹配
优化的目标：当前帧相对于上一帧的位姿
Ceres 会优化：para_q:旋转四元数； para_t:平移向量

3.后端：
laserMapping.cpp   ----后端建图优化模块
做的内容是  当前帧特征点 vs 局部地图特征点   ----scan to map匹配
① 当前点变换到地图坐标系
② 局部地图维护：目的：减少计算量，提高实时性，只匹配当前位置附近的有效结构

4.后端角点匹配：PCA找线
在后端，当前角点不是简单找两个点，而是在局部地图角点里找 5 个近邻点，然后判断这 5 个点是否形成线结构。

5.后端优化平面点匹配：最小二乘拟合平面
当前平面点会在局部地图平面点中找 5 个近邻点，然后拟合平面。

6.当前帧加入地图并降采样
优化完成后，当前帧特征点会加入地图。

```

fast_lio需要理解的问题：

```
1. 为什么需要 IMU？
2. IMU 如何预测位姿？
3. 点云去畸变怎么做？
4. ESKF 状态量有哪些？
5. 雷达观测残差是什么？
6. 局部地图怎么维护？
7. ikd-tree 是做什么的？
8. 为什么 FAST-LIO 适合 Livox？
```

<img width="679" height="663" alt="image" src="https://github.com/user-attachments/assets/2ea4aedd-3ad5-44ea-acf4-5e89f4889e26" />


## FAST_LIO 理解的过程：
1.读源码链路：
```
1.全局变量和参数读取
2.ROS订阅发布
3.点云回调函数  ----mid360 ----> livox_pcl_cbk(...)
4.IMU回调函数
5.数据同步函数：作用：把lidar_buffer 和 imu_buffer 按时间配成一组Measures
6.主循环 while(ros::ok())
7.IMU处理/点云去畸变   ---完成imu传播和点云去畸变 输出feats_undistort
8.ESKF更新
9.地图更新 ----->map_incremental()
10.发布odometry/path/点云

```




2.主循环逻辑：
```
接收回调数据
  ↓
同步一帧雷达+一段IMU
  ↓
IMU预测 + 点云去畸变
  ↓
裁剪局部地图
  ↓
点云降采样
  ↓
如果地图没有初始化，先建第一帧地图
  ↓
ESKF 迭代更新位姿
  ↓
把当前帧点云加入id-tree 地图
  ↓
发布odometry/path/点云


```

3.一帧imu数据大概长这样：
```
时间戳: 10.01 秒
角速度: wx, wy, wz
加速度: ax, ay, az
姿态: qx, qy, qz, qw
```

一包IMU数据，包含：

```
msg->header.stamp       //时间戳
msg->header.frame_id    //坐标系名字

msgs->angular_velocity.x  //x方向角速度
msgs->angular_velocity.y  //y方向角速度
msgs->angular_velocity.z  //z方向角速度

msg->linear_acceleration.x   //x方向线加速度
msg->linear_acceleration.y   //y方向线加速度
msg->linear_acceleration.z   //z方向线加速度

msg->orientation.x       //四元数姿态
msg->orientation.y       
msg->orientation.z       
msg->orientation.w       


```

imu数据储存起来  用来做：
```
imu预积分
点云去畸变
状态预测
ESKF前向传播
```

## 4.FAST-LIO 不是单独处理雷达，也不是单独处理 IMU，而是把一帧雷达和对应时间段内的 IMU 数据**同步**起来处理。
数据同步函数 bool sync_packages(MeasureGroup &meas)  处理步骤：
```
① 如果任一缓存为空，直接失败：
```
```c++
if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
}
```

```
② 取队首雷达帧：
```
```c++
if(!lidar_pushed)
{
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
//lidar_pushed 是一个状态标志，防止当前雷达帧还没等到足够 IMU 数据时被重复取出或弹出。
```

```
③ 估计这一帧 LiDAR 的结束时间：
```
```c++
lidar_end_time = meas.lidar_beg_time
               + meas.lidar->points.back().curvature / double(1000);
//这里很关键：FAST-LIO 把每个点相对当前帧起始时刻的时间，存在点的 curvature 字段里，单位近似是毫秒。所以最后一个点的 curvature / 1000 就是这一帧扫描持续时间。
//结束时间 = 起始时间 + 扫描这一帧花掉的时间  如12:00：00开始跑步  花了10s跑完---->结束时间：12：00：10；
```
```
④ 检查 IMU 是否已经覆盖到这帧 LiDAR 的结束时间：
```
```c++
if (last_timestamp_imu < lidar_end_time)
{
    return false;
}
//如果最新 IMU 时间还早于当前 LiDAR 帧结束时间，说明 IMU 数据还不够，暂时不处理这一帧，等下一次回调进来。
```

```
⑤ 取出所有时间小于等于 lidar_end_time 的 IMU：
```
```c++
//这一步会把当前雷达帧期间需要的 IMU 数据全部放到：meas.imu
double imu_time = imu_buffer.front()->header.stamp.toSec();
meas.imu.clear();
while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
{
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if(imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
}

```
```
⑥ 弹出已经同步完成的雷达帧：
```
```c++
lidar_buffer.pop_front();
time_buffer.pop_front();
lidar_pushed = false;
return true;
//返回 true 后，主循环才会继续做：p_imu->Process(Measures, kf, feats_undistort); (imu处理和点云去畸变)

```

<img width="476" height="385" alt="image" src="https://github.com/user-attachments/assets/55c274f0-18e9-4f88-bd0b-76e5cc7a7bb7" />


## 5.一些注释：
```
1)tag 是Liovx点云里对点质量/回波类型的一种标记，这里是在筛选一些不好的点，只保留某些类型的点

2)curvature 当前点相对这一帧点云开始时刻的时间
pl_full[i].curvature = 50   ------>表示这个点是当前帧开始后50ms扫到的

3)blind 雷达盲区阈值

4)pl_surf 最终保留下来的有效点云。

5)N_SCANS 雷达总共有多少条扫描线
最终用于匹配的点云：

6)feats_undistort 去畸变之后的点云
  feats_down_body 下采样之后的点云  ------>这个是真正进入地图匹配/ESKF 的

7)pl_full 完整点云/临时完整点云
  pl_surf 面点/普通有效点   -------最终保留下来、后面要参与匹配/建图/ESKF 更新的有效点云
  pl_corn 角点/边缘点   ------可能存在但不一定大量使用

8)ptr 是 preprocess 输出的当前帧点云，存在 lidar_buffer 里

9)Measures.lidar 是 sync_packages 从 lidar_buffer 取出的当前帧点云

10)feats_undistort 是 p_imu->Process 对 Measures.lidar 做 IMU 去畸变后的点云

11)feats_down_body 是 feats_undistort 经过 VoxelGrid 下采样后的点云，也是后面匹配主要使用的点云
```


## 6. 点云预处理过程：

```
原始 Livox 点云 msg
→ 遍历每个点
→ 判断扫描线和 tag
→ 按 point_filter_num 降采样
→ 复制 x/y/z/反射强度/时间
→ 去掉重复点和盲区点
→ 存入 pl_surf
```

```
原始点云格式是什么？
livox_ros_driver::CustomMsg
最终用于匹配的点云是什么？
两个 一个是去畸变之后的点云feats_undistort，一个是降采样之后的点云feats_down_body
点云里的每个点有没有时间信息？
有。时间信息被塞在 PointType::curvature 里，不是真的曲率，而是“点在这一帧内的相对时间”。
blind 参数在哪里用？
blind 是近距离滤波阈值，单位米。在avia_handler() 里过滤近点，在give_feature() 里跳过近点
在plane_judge()、edge_jump_judge() 等特征判断里也会用
voxel/downsample 有没有做？
有，做了，而且是后端匹配前做的。
```

点云的格式转换
```
原始可能是：
msg->points[i].x
msg->points[i].y
msg->points[i].z
msg->points[i].reflectivity
msg->points[i].offsettime

转换成FAST_LIO/PCL常用的点类型:
pl_full[i].x = msg->points[i].x
pl_full[i].y = msg->points[i].y
pl_full[i].z = msg->points[i].z
pl_full[i].intensity = msg->points[i].reflectivity       (强烈、强度)     (反射) 
pl_full[i].x = msg->points[i].x
pl_full[i].curvature = msg->points[i].offset_time / float(1000000)

```
涉及的一些转换关系过程
```
原始Livox消息msg
    |
    ↓
pl_full 格式转换后的当前帧完整点云
    |
    |  筛选：tag、line、降采样、去重复、去盲区
    ↓
pl_surf 最终保留下来的有效点/面点
    |
    ↓
后端去畸变、地图匹配、ESKF更新

```





