# XJTU-RMV-Task03
### 视觉组第三次作业
#### Ceres 应用：拟合小球弹道

**拟合模型——假设小球弹道满足：**

$$
x(t)=x_0+{v_{x0}\over k}(1-e^{-kt})
$$

$$
y(t)=y_0+{(v_{y0}+{g\over k})\over k}(1-e^{-kt})-{g\over k}t
$$

-------------

拟合结果：

```
vx0: 244.564
vy0: 340.795
k: 0.0404434
g: 486.6
```

更多运行结果详见 `outputs/results.txt`

![运行结果截图](outputs/results.png)