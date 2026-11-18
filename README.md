# 基于虚拟载具的 Kalman 预测编队项目

本项目以 `TVF_ver260112_formal_ver2.m` 为控制律基线，在不改变原程序从 `k=2` 开始的时序和初始状态含义的前提下，为领导者加入 `[x;y;v]` Kalman 状态估计、带噪声位置观测、后验领导者及可选的 `0.1 s` 前视输出。

滤波器只递推位置和纵向速度 `[x;y;v]`。航向角 `theta` 与角速度 `w` 直接传递，不进入当前 Kalman 状态。编队控制默认使用当前后验领导者；`0.1 s` 前视领导者主要用于观察，不参与当前融合过程。

## 建议运行顺序

1. 直接观察完整算法与动画：运行 `run_kalman_prediction_virtual_agent_demo.m`。
2. 只进行数值仿真并保存数据：运行 `run_kalman_prediction_simulation_only.m`。
3. 运行结构化协方差对照实验：运行 `run_kalman_prediction_simulation_pxy_zero.m`。
4. 比较完整协方差与 `p_xy=0` 结果：运行 `compare_pxy_zero_kalman_results.m`。
5. 根据保存的 MAT 生成动画：选择下文两个正式画图程序之一。

程序均根据自身路径定位项目目录。需要预先加入项目路径时，可以先运行：

```matlab
project_root = setup_kalman_prediction_project_path();
```

## 主要 MATLAB 文件

### `run_kalman_prediction_virtual_agent_demo.m`

完整演示主程序。它在一个文件中完成：

- 真实领导者和四个跟随者的状态更新；
- 唯一公共带噪声位置观测的生成；
- `[x;y;v]` Kalman 预测与测量更新；
- 当前后验领导者和 `0.1 s` 前视领导者的生成；
- 四个虚拟编队参考载具的计算；
- 基于 ver2 TVF 控制律的四跟随者控制；
- 动画及概率融合过程显示。

该文件适合单步调试和整体理解算法数据流。

### `run_kalman_prediction_simulation_only.m`

从完整主程序拆出的“只运行主循环”版本。它保留相同的仿真时序、Kalman 递推和控制算法，但不负责生成动画，运行结束后把完整变量保存到 `saves/`：

```text
kalman-test-YY-MM-DD-HHMMSS.mat
```

该版本用于把耗时的数值仿真与动画渲染分离，便于重复调整画图而不重新计算轨迹。

### `run_kalman_prediction_simulation_pxy_zero.m`

`p_xy=0` 结构化协方差对照版本，来源于 `run_kalman_prediction_simulation_only.m` 的副本。除以下约束外，其余参数、随机种子、时序和控制过程保持一致：

```matlab
P(1,2) = 0;
P(2,1) = 0;
```

该约束在每次预测和更新后执行，用来检验 `x`、`y` 位置误差耦合是否会在当前简单观测结构中传播不必要的噪声。输出文件为：

```text
kalman-test-pxy-zero-YY-MM-DD-HHMMSS.mat
```

该实验不删除 `p_xv` 和 `p_yv`，所以位置创新仍可通过交叉协方差修正纵向速度。

### `compare_pxy_zero_kalman_results.m`

自动读取 `saves/` 中最新的普通仿真与 `p_xy=0` 仿真，生成：

- 逐时刻误差数据 CSV；
- 汇总指标 CSV；
- 对照 MAT；
- Kalman 先验预测值与真实值对比图；
- 完整协方差与 `p_xy=0` 的误差—时间对比图。

### `generate_kalman_prediction_animation_from_save.m`

第一版正式动画程序。默认读取 `saves/` 中最新的 `kalman-test-*.mat`，生成传统二维概率曲线动画：

- 左侧显示领导者、跟随者及可选观测量；
- 右侧依次显示先验与观测、融合后验、前视分布；
- 可以在文件开头选择显示 `X` 或 `Y` 方向的一维高斯分布。

该程序适合逐阶段阅读单轴概率融合过程。

### `generate_kalman_fusion_combined_animation_from_pxy_zero_save.m`

第二版正式动画程序，也是当前 `p_xy=0` 数据的主要可视化入口。默认直接读取最新的 `kalman-test-pxy-zero-*.mat` 并生成完整 GIF：

- 左侧：真实领导者、四个跟随者和可选前视领导者；
- 右上：红色先验、蓝色观测和紫色后验的 XY 二维高斯三维叠加图；
- 右中：速度先验、后验位置窗口推导速度、现有 Kalman 速度后验和真实速度参考；
- 右下：后验位置与真实位置的误差模值—时间曲线。

正式配置使用 `8 sigma` 显示范围，并把三个位置分布的均值间距视觉放大 `8` 倍，以便识别三个峰顶。该放大只影响显示，不修改 MAT 中的均值、协方差或滤波结果。

生成文件写入 `saves/`：

```text
kalman-test-pxy-zero-fusion-combined-YY-MM-DD-HHMMSS.gif
```

### `setup_kalman_prediction_project_path.m`

把项目根目录和 `auxiliary/` 加入 MATLAB 路径，适合命令行调试、打包或从其他目录调用程序。

## 文件夹

### `auxiliary/`

主程序和画图程序共用的短辅助函数：

- `clamp_value_to_limits_helper.m`：数值限幅；
- `ensure_positive_variance_helper.m`：保证绘图方差为正；
- `evaluate_normal_probability_density.m`：手算一维正态概率密度；
- `plot_color_and_transparency_helper.m`：MATLAB R2019a 下的颜色与透明度填充；
- `wrap_angle_to_pi_helper.m`：航向角折叠到 `[-pi,pi]`。

### `saves/`

正式数值结果和动画输出目录，主要包括：

- 普通仿真 MAT；
- `p_xy=0` 仿真 MAT；
- 协方差对照实验的 CSV、MAT、FIG 和 PNG；
- 两个正式画图程序生成的 GIF。

动画程序默认按修改时间读取相应类型的最新 MAT，也可以在程序开头指定文件名。

### `tex/`

Kalman 数学说明文档及其编译文件：

- `kalman_gaussian_matrix_fusion_explanation.tex`：当前 TeX 源文档；
- `kalman_gaussian_matrix_fusion_explanation.pdf`：编译后的正式 PDF；
- `assets/`：PDF 使用的流程图和概率融合截图；
- `kalman_oldversion_note.pdf`：早期说明文档存档；
- 其余 `.aux/.log/.toc/.xdv/.synctex.gz` 等为 LaTeX 编译产物。

TeX 必须使用工作区 `.vscode/settings.json` 中的 VS Code LaTeX Workshop 配置编译：

```powershell
C:\texlive\2026\bin\windows\latexmk.exe -xelatex -synctex=1 -interaction=nonstopmode -file-line-error kalman_gaussian_matrix_fusion_explanation.tex
```

### `results/`

完整演示主程序产生的历史概率融合 MAT/GIF。静态尺度预览、烟雾测试图和样式比较图属于临时文件，不作为正式结果保留。

### `baseline/`

用于保存或追溯原始 TVF 基线材料；当前正式算法以 `TVF_ver260112_formal_ver2.m` 为设计基线。

### `flowchart/`

项目流程图及 Mermaid 等可编辑流程说明的预留目录。

## 其他文档

- `卡尔曼预测虚拟领导者_实施任务清单.md`：开发过程、设计约束和待办事项记录；
- `README_项目说明.md`：早期快速运行说明；
- 本文件 `README.md`：当前项目结构和正式文件用途总览。

## 关键设计约束

- 主循环仍从 `k=2` 开始，初始状态位于索引 `1`；
- 不改变 ver2 TVF 控制器的原始仿真时序；
- 速度状态更新中的时间尺度与位移状态更新保持一致；
- `theta`、`w` 不进入当前 `[x;y;v]` Kalman 滤波；
- 默认控制输入使用当前后验领导者，不使用 `0.1 s` 前视位置；
- 后验位置窗口推导速度目前只用于第二版动画诊断，不会再次融合进同一时刻的 Kalman 后验。
