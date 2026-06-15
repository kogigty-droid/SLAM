# 三种slam框架 ：A-LOAM、FAST_LIO、ORB_SLAM
```
共同点：
本质上都是在估计：
```
$$
\text{本质上都是在估计：} =
\left\{
\begin{array}{ll}
\text{点到线残差}, & \text{角点} \\
\text{点到平面残差}, & \text{平面点}
\end{array}
\right.
$$

