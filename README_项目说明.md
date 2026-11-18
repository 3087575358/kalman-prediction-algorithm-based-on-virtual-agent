# 基于虚拟载具的卡尔曼预测编队算法

## 直接运行

在 MATLAB 中打开并运行：

```matlab
run_kalman_prediction_virtual_agent_demo
```

主文件会根据自身位置自动加入 `auxiliary`，因此不要求 MATLAB 当前目录必须是项目目录。

## 路径模式

调试、批量运行或打包前可以先执行：

```matlab
project_root = setup_kalman_prediction_project_path();
run_kalman_prediction_virtual_agent_demo
```

## 核心数据流

1. 真实领导者位置生成唯一的公共带噪声观测。
2. 上一时刻后验 `[x;y;v]` 在直接传递的 `theta,w` 下通过非完整运动学生成当前先验分布。
3. 当前先验与观测分布融合，生成当前后验分布。
4. 当前后验向前传播 0.1 s，生成算法前视输出。
5. 默认使用当前融合后验计算四个编队参考载具；前视输出只作动画观察。
6. ver2 TVF 控制器根据编队参考载具控制四个跟随者。

0.1 s 前视结果不参与当前时刻的卡尔曼融合；它是融合完成后的预测输出。若需要试验前视控制，可把主文件顶部的 `preview_used_for_control` 改为 `1`。

滤波器只递推 `[x;y;v]`。`theta` 和 `w` 直接传递，不进行 Kalman 滤波；`vx`、`vy` 由 `v` 和 `theta` 解析得到。MAT 数据同时保留 `state_x/state_y` 等旧字段，供动画读取兼容，但它们不是两套独立滤波器。

## 默认动画

- 左侧默认显示真实领导者、0.1 s 前视领导者和四个跟随者。
- 四个编队参考载具不显示。
- 带噪声观测和当前后验领导者由主文件顶部开关控制。
- 右侧依次显示先验与观测、融合后验、0.1 s 前视分布。

完整概率数据和 GIF 输出在 `results` 文件夹。

## 仿真与动画分离运行

只运行主循环并保存 MAT：

```matlab
run_kalman_prediction_simulation_only
```

存档写入 `saves`，命名格式为：

```text
kalman-test-YY-MM-DD-TIMENOW.mat
```

读取存档并生成动画：

```matlab
generate_kalman_prediction_animation_from_save
```

动画程序默认读取 `saves` 中最新的 `kalman-test-*.mat`，也可在文件开头设置
`selected_mat_filename`。生成的 GIF 同样写入 `saves`：

```text
kalman-test-YY-MM-DD-TIMENOW.gif
```

动画文件开头可以分别控制左侧智能体、右侧概率分布、带噪声观测、后验领导者、
0.1 s 前视领导者、四个编队参考载具和四个跟随者的显示。
