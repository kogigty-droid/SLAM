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

## 7.IMU_Processing.hpp
三个主要职责：
```
IMU_init()       初始化重力、陀螺仪 bias、协方差、外参
UndistortPcl()  用 IMU 预测轨迹，并对一帧点云做运动补偿
Process()       外部入口，决定当前是初始化还是正常去畸变
```
```
其中的有做在线均值和方差估计
mean_acc 平均加速度用来估计重力方向
mean_gyr 平均角速度，用来估计陀螺仪bias
cov_acc 加速度噪声估计
cov_pyr 角速度噪声估计
```
<img width="566" height="295" alt="image" src="https://github.com/user-attachments/assets/c40edfc4-d95d-49d2-b165-5dad7eac6844" />

<img width="699" height="371" alt="image" src="https://github.com/user-attachments/assets/b0a39bf6-ee64-4963-a8a3-9a25b45b8986" />


```
init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
grav 根据平均加速度估计重力方向

init_state.bg  = mean_gyr;
bg 陀螺仪bias = 禁止时平均角速度

init_state.offset_T_L_I = Lidar_T_wrt_IMU;  //LiDAR 到 IMU 平移外参
init_state.offset_R_L_I = Lidar_R_wrt_IMU;  //LiDAR 到 IMU 平移外参

状态协方差矩阵：P  它表示滤波器对当前状态估计有多不确定。
P 越大：说明我对这个状态越不自信，后面更愿意被观测修正
P 越小：说明我对这个状态越自信，后面不太容易被观测大幅修改
```
总结：
```
IMU 初始化：用静止或近似静止的前几帧 IMU 估计重力方向和陀螺 bias。

IMU 预测：用相邻 IMU 的平均 acc / gyro，调用 kf_state.predict(dt, Q, in) 推进状态。

点云去畸变：利用点的 curvature 时间戳，计算每个点采集时刻的位姿，把点补偿到当前帧末端坐标系。
```

# 8. 主要函数之：构造点到平面残差方程：h_share_model()
  该函数在这个函数kf.update_iterated_dyn_share_modified(...)的执行过程中被调用
## 作用：把当前帧点云和局部地图做匹配，计算点到平面的残差，并构造 ESKF 更新需要的 H 矩阵和残差 h。


```
相关变量：
feats_down_body     当前帧去畸变后、降采样后的点云 坐标系是 body/IMU 或雷达体坐标相关坐标
feats_down_world     把feats_down_body通过当前状态s 转到世界坐标系之后的点云
feats_down_size     feats_down_body 的点数
point_body    当前点在body坐标系下的坐标
point_world    当前点转换到世界坐标系之后的坐标
Nearest_Points[i]    第i个点在ikd-tree地图中搜索到的最近邻点集合
pointSearchSqDis    最近邻点的距离一般是平方距离
point_selected_surf[i]    第i个点是否被认为是有效的平面匹配点
pabcd    拟合出来的平面参数：a,b,c,d   这里：平面方程：a*x+b*y+c*z+d=0
pd2    当前点到拟合平面的有符号距离/残差
normvec    临时保存每个点对应的平面法向量和残差
            x,y,z    平面法向量
            intensity     点到平面的距离 pd2
laserCloudOri    最终筛选出来的有效点云 body坐标系
corr_normvect    和laserCloudOri 一一对应的平面法向量和残差
effct_feat_num    有效的匹配点数量
ekfom_data.h_x    ESKF的观测雅可比H
ekfom_data.h    ESKF的观测残差
extrinsic_est_en    是否在线估计雷达-IMU外参
```
```c++
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{   /*
     这里：s 表示当前ESKF状态 里面有位置、姿态、速度、IMU bias、重力、雷达到IMU外参等
           ekfom_data: ESKF更新时用的数据结构，这个函数里面往往会填：
                        h_x 观测雅可比矩阵 H
                        h 残差向量
                        valid 当前观测是否有效
    */
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN                                       //如果开启 MP_EN，这里会用 OpenMP 多线程并行处理点云。
        omp_set_num_threads(MP_PROC_NUM);                //每个点可以独立做坐标变换，最近邻搜索，平面拟合，残差计算
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];   //直接引用当前帧降采样点云里的第 i 个点
        PointType &point_world = feats_down_world->points[i];  //直接引用世界系点云数组里的第 i 个位置

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);  //把 PCL 点转成 Eigen 三维向量。
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        //s.rot：从 body/IMU 坐标系 转到 world 坐标系 的旋转  s.pos：当前IMU/body系原点在世界坐标系下的位置(平移)
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS); //创建一个长度为 NUM_MATCH_POINTS 的距离数组

        auto &points_near = Nearest_Points[i];  //Nearest_Points[i] 是第 i 个点对应的最近邻点集合
        //在地图里找最近邻
        if (ekfom_data.converge)    //converge  集中 收敛   只有在迭代收敛的情况下才找最近邻
        {
            /** Find the closest surfaces in the map **/
            /**在局部地图 ikdtree 里，为 point_world 找 NUM_MATCH_POINTS 个最近邻点**/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;

            /**
            上面这个三目运算符的运用  等价于：
            if(points_near.size() < NUM_MATCH_POINTS)   //如果邻居数量不够，丢弃。
            {
                point_selected_surf[i] = false;  
            }
            else if(pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)   //如果第 k 个邻居太远，丢弃。
            {
                 point_selected_surf[i] =false;
            }
            else
            {
                point_selected_surf[i] = true;   //否则认为这个点可以尝试拟合平面。
            }

            **/
        }

        if (!point_selected_surf[i]) continue;   //如果这个点不可用 就跳过

        //用最近邻点拟合平面
        VF(4) pabcd;   //四维向量
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))    //esti_plane(...) 在 include/common_lib.h，核心是最小二乘拟合平面。
        {
            //FAST-LIO 就是想通过 ESKF 更新，让这些 pd2 尽量接近 0。
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            //p_body.norm() 点到雷达原点的距离。
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());   //残差越小，s 越接近 1；

            if (s > 0.9)     //这里的s是匹配质量分数
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);   
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;     //如果没有有效点，观测无效
        ROS_WARN("No Effective Points! \n");
        return;
    }

    //统计平均残差和耗时
    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        //<< 是 Eigen 矩阵赋值语法，叫 逗号初始化 comma initializer

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);    //s.rot.conjugate()可以理解为旋转矩阵的逆  世界系下的法向量转回 body/IMU 相关坐标系
                                                //用来表示外参平移对残差的影响
        V3D A(point_crossmat * C);   //A 是残差对当前姿态扰动的雅可比相关项。
        if (extrinsic_est_en)    //是否在线估计lidar到imu的外参
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;      //残差 = 观测值-预测值  说明这里观测值是0
    }
    solve_time += omp_get_wtime() - solve_start_;
}
```



