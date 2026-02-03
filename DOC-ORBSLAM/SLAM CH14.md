# ch3

<img width="1070" height="850" alt="image" src="https://github.com/user-attachments/assets/221153fd-af4b-4f51-af84-487ee6abc5f5" />

## 学习路线建议：

<img width="1099" height="335" alt="image" src="https://github.com/user-attachments/assets/0cd6adab-223d-4f79-880f-182fbab6206b" />



# ch5 

## 去畸变

<img width="1635" height="705" alt="image" src="https://github.com/user-attachments/assets/a9c59153-2f03-4205-8807-090dd309e309" />

## 拼合的点云地图

<img width="1105" height="930" alt="image" src="https://github.com/user-attachments/assets/142f1700-01f7-4b53-9b61-6edcda1cc687" />

## 一些约定俗成：

<img width="633" height="174" alt="image" src="https://github.com/user-attachments/assets/3226fab8-8fd9-4cb4-8179-69df24d0007c" />



# ch6
### 手写高斯牛顿法

<img width="1800" height="1030" alt="image" src="https://github.com/user-attachments/assets/b5ab0de9-63e8-4b59-a2c5-f053b8a2c5f1" />

### ceres库使用

<img width="999" height="329" alt="image" src="https://github.com/user-attachments/assets/457ed082-2c5c-4531-87ee-eb486fbc7117" />

### 图的一些理解 顶点---优化变量   边----误差项

<img width="729" height="239" alt="image" src="https://github.com/user-attachments/assets/53177e17-f305-42c4-9f21-076cf7d35674" />

### 边的定义

```cpp
// 这一整个 struct 就是一条“边”的定义
struct CURVE_FITTING_COST {
    // 这里存的是边的【观测值 z】
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    template<typename T>
    bool operator()(
        const T* const abc, // 这里是边连接的【顶点变量】
        T* residual         // 这里计算的是【误差项 e】
    ) const {
        // residual = [观测值] - [通过模型推算的预测值]
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
    const double _x, _y;
};
```

#### main函数中：

```cpp
problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
), 
    nullptr, // 这里是【鲁棒核函数】，现在没用
    abc      // 这里指定这条边连在哪几个【顶点】上
);
```

### g2o库
### 利用g2o库写一个高斯-牛顿方法进行梯度下降

<img width="998" height="946" alt="image" src="https://github.com/user-attachments/assets/405ce25c-9058-4172-bc79-e243f8b0ed9b" />


