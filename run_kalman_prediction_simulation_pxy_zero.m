%% Pxy 强制为零实验：基于虚拟载具的卡尔曼预测编队跟踪
% 基线：TVF_ver260112_formal_ver2.m
% 设计约束：保留原程序从 k=2 开始的时序，不改变初始状态的含义。
% 对照实验约束：每次预测和更新后都强制 P(1,2)=P(2,1)=0，其余算法保持不变。

% 可选运行参数由预览入口传入；普通直接运行时使用下方默认值。
if exist('kalman_run_options', 'var') && isstruct(kalman_run_options)
    runtime_options = kalman_run_options;
else
    runtime_options = struct();
end
clearvars -except runtime_options;
close all; clc;

%% 01 项目路径：既支持直接运行，也支持先调用 setup_kalman_prediction_project_path
script_full_path = mfilename('fullpath');
if isempty(script_full_path)
    project_root = pwd;
else
    project_root = fileparts(script_full_path);
end
auxiliary_folder = fullfile(project_root, 'auxiliary');
saves_folder = fullfile(project_root, 'saves');
if ~isfolder(auxiliary_folder)
    error('没有找到辅助函数目录：%s', auxiliary_folder);
end
if ~isfolder(saves_folder)
    mkdir(saves_folder);
end
addpath(auxiliary_folder);

%% 02 仿真、滤波和动画配置
convergence_performance = 0;
generate_animation = 0;
save_static_preview_frames = 0;
static_preview_times = [];

% 左侧动画显示开关：四个编队参考载具始终参与计算，但不在左侧显示。
show_noisy_leader_observation = 0;
show_posterior_leader = 0;
show_preview_leader = 1;

% 1 表示 0.1 s 前视结果参与编队参考计算；0 表示仅用于动画观察。
preview_used_for_control = 0;
probability_display_axis = 'x';       % 可改成 'y'
preview_horizon = 0.1;                % 前视时间，单位 s

% 手写恒速卡尔曼滤波参数。实际注入噪声与滤波器采用的 R 分开设置。
injected_position_noise_std_x = 0.05; % 实际注入的 X 位置噪声标准差，单位 m
injected_position_noise_std_y = 0.05; % 实际注入的 Y 位置噪声标准差，单位 m
kalman_assumed_observation_std_x = 0.08; % 滤波器假定的 X 观测标准差，单位 m
kalman_assumed_observation_std_y = 0.08; % 滤波器假定的 Y 观测标准差，单位 m
process_acceleration_std = 20.0;      % 单一 [x,y,v] 模型的线加速度过程噪声
random_seed = 20261110;
rng(random_seed, 'twister');

% 概率曲线视觉参数集中配置，主动画中不散落 RGB 和透明度常量。
prior_distribution_color = [0.85, 0.15, 0.15];
observation_distribution_color = [0.15, 0.35, 0.90];
posterior_distribution_color = [0.10, 0.65, 0.25];
preview_distribution_color = [0.55, 0.00, 0.00];
active_distribution_opacity = 0.70;
context_distribution_opacity = 0.18;

% 仿真数据写入 saves，命名格式为 kalman-test-YY-MM-DD-TIMENOW.mat。
timestamp = datestr(now, 'yy-mm-dd-HHMMSS');
mat_filename = fullfile(saves_folder, sprintf('kalman-test-pxy-zero-%s.mat', timestamp));

% 原 ver2 仿真参数。
dt = 0.01;
T = 20;
t = 0:dt:T;
N = length(t);
frame_stride = 10;

% 预览入口可以覆盖少量运行参数；直接运行主文件时这些条件不会触发。
if isfield(runtime_options, 'generate_animation')
    generate_animation = logical(runtime_options.generate_animation);
end
if isfield(runtime_options, 'save_static_preview_frames')
    save_static_preview_frames = logical(runtime_options.save_static_preview_frames);
end
if isfield(runtime_options, 'static_preview_times')
    static_preview_times = runtime_options.static_preview_times;
end
if isfield(runtime_options, 'total_simulation_time')
    T = runtime_options.total_simulation_time;
    t = 0:dt:T;
    N = length(t);
end
if isfield(runtime_options, 'preview_used_for_control')
    preview_used_for_control = logical(runtime_options.preview_used_for_control);
end

v_min = -50; v_max = 50;
w_min = -1; w_max = 1;

%% 时变信号下小车的编队位置
% 这里着重介绍一下我们的命名规则
% hx_1_hat是我的领导者1和跟随者的设定距离输入（领导者本地坐标系下），通过下面的函数进行设定
% hx_1_arr是实际跟随者1和领导者的实际期望距离（世界坐标系下），在主循环中通过每一次计算得到，并存储在hx_1_arr数组当中
% 编队设定距离
hx_1_hat = @(t) 30 * cos(0.1 * t);   hy_1_hat = @(t) 30 * sin(0.1 * t);
hx_2_hat = @(t) -30 * sin(0.1 * t);   hy_2_hat = @(t) 30 * cos(0.1 * t);
hx_3_hat = @(t) -30 * cos(0.1 * t);   hy_3_hat = @(t) -30 * sin(0.1 * t);
hx_4_hat = @(t) 30 * sin(0.1 * t);   hy_4_hat = @(t) -30 * cos(0.1 * t);

% 实际的小车运行中为了保证编队的相对收敛，计算编队速度与领导者速度叠加所带来的“最大抛射速度”与“最大抛射加速度”
% 最大抛射速度就是跟随者实际运动过程中，小车的物理极限
% 对应计算就能得到编队收敛的前提下，领导者的速度极限
    % v_formation = diff(sqrt(hx_1_hat^2 + hy_1_hat^2), t);
    % a_formation = diff(v_formation, t);
    % if a_formation_tau == 0
    %     v_formation_max_ = v_formation_tau;
    % end
    % %vp_max = abs(v_formation_max) + abs(v0_max);
    % %vp_max = 50;
    % v0_max = abs(vp_max) - abs(v_formation_max);

%% 设置领导者的运动轨迹和速度
v0 = @(t) -min(2*t - 20, 0);          % 运动到10秒时停止
%v0 = @(t) 0;                         % 保持静止用于观察队列的变化
%v0 = @(t) 20 * sin(t);
%v0 = @(t) 20 - t;
%v0 = @(t) ((10 - t) / 2 ) ^ 2  
w0 = @(t) (10 - t)/20 * cos(0.1 * t);
%w0 = @(t) 0.8;
%w0 = @(t) v0(t)/R(t);                % 利用曲率计算角速度

% 领导者本地坐标系下小车的期望距离
hx_1_arr = zeros(1,N); hy_1_arr = zeros(1,N);
hx_2_arr = zeros(1,N); hy_2_arr = zeros(1,N);
hx_3_arr = zeros(1,N); hy_3_arr = zeros(1,N);
hx_4_arr = zeros(1,N); hy_4_arr = zeros(1,N);


% virtual局部坐标系下的 跟踪误差
error_1 = zeros(3,N); 
error_2 = zeros(3,N); 
error_3 = zeros(3,N); 
error_4 = zeros(3,N);

% 控制器参数
k1_arr_f1 = zeros(1,N);k2_arr_f1 = zeros(1,N);k3_arr_f1 = zeros(1,N);k4_arr_f1 = zeros(1,N);
k1_arr_f2 = zeros(1,N);k2_arr_f2 = zeros(1,N);k3_arr_f2 = zeros(1,N);k4_arr_f2 = zeros(1,N);
k1_arr_f3 = zeros(1,N);k2_arr_f3 = zeros(1,N);k3_arr_f3 = zeros(1,N);k4_arr_f3 = zeros(1,N);
k1_arr_f4 = zeros(1,N);k2_arr_f4 = zeros(1,N);k3_arr_f4 = zeros(1,N);k4_arr_f4 = zeros(1,N);

%% ==================== 创建状态变量 ====================
% 生成领导者
leader = define_agent_state(N);
% 生成跟随者状态
follower_1 = define_agent_state(N);
follower_2 = define_agent_state(N);
follower_3 = define_agent_state(N);
follower_4 = define_agent_state(N);
% 生成每个跟随者的幽灵载具/追踪目标的信号
virtual_1 = define_agent_state(N);
virtual_2 = define_agent_state(N);
virtual_3 = define_agent_state(N);
virtual_4 = define_agent_state(N);

%% 03 卡尔曼概率融合数据预分配
leader_observation.x = zeros(1, N);
leader_observation.y = zeros(1, N);
leader_prior = define_leader_estimate_state(N);
leader_posterior = define_leader_estimate_state(N);
leader_posterior.kalman_gain = zeros(3, 2, N);
leader_posterior.kalman_gain_x = zeros(2, N); % 兼容旧动画读取
leader_posterior.kalman_gain_y = zeros(2, N); % 兼容旧动画读取
leader_preview = define_leader_estimate_state(N);
leader_preview.variance_x = zeros(1, N);
leader_preview.variance_y = zeros(1, N);

% 四个 virtual 的 offset 速度诊断数组：行=[vx; vy; speed]，列=virtual编号，页=k。
virtual_offset_velocity = zeros(3, 4, N);
%% 初始状态赋值
% 领导者初始位置与角朝向
leader.x(1) = 0; leader.y(1) = 0; leader.theta(1) = 0; leader.v(1) = v0(0); leader.w(1) = w0(0);

% Stage 01 初始化：只生成一份二维公共观测，供单一 [x;y;v] 滤波器使用。
[leader_observation.x(1), leader_observation.y(1)] = ...
    stage_01_generate_noisy_leader_observation( ...
    leader.x(1), leader.y(1), injected_position_noise_std_x, injected_position_noise_std_y);

% 初始单一状态为 [x;y;v]；theta、w 直接传递，不参与融合。
initial_filter_state = [leader_observation.x(1); leader_observation.y(1); leader.v(1)];
initial_filter_covariance = diag([kalman_assumed_observation_std_x^2, ...
    kalman_assumed_observation_std_y^2, 25]);
leader_posterior = store_leader_estimate_fields(leader_posterior, 1, ...
    initial_filter_state, initial_filter_covariance, leader.theta(1), leader.w(1));
leader_prior = store_leader_estimate_fields(leader_prior, 1, ...
    initial_filter_state, initial_filter_covariance, leader.theta(1), leader.w(1));

% Stage 04 初始化：采用同构圆弧预测，但 theta、w 不进入卡尔曼更新。
[preview_state_1, preview_covariance_1] = stage_04_predict_leader_position_ahead( ...
    initial_filter_state, initial_filter_covariance, leader.theta(1), leader.w(1), ...
    preview_horizon, process_acceleration_std);
preview_theta_1 = wrap_angle_to_pi_helper(leader.theta(1) + leader.w(1) * preview_horizon);
leader_preview = store_leader_estimate_fields(leader_preview, 1, ...
    preview_state_1, preview_covariance_1, preview_theta_1, leader.w(1));
leader_preview.variance_x(1) = preview_covariance_1(1,1);
leader_preview.variance_y(1) = preview_covariance_1(2,2);

% 幽灵载具初始位置
virtual_1.x(1) = hx_1_hat(0);virtual_1.y(1) = hy_1_hat(0); virtual_1.theta(1) = 0;
virtual_2.x(1) = hx_2_hat(0);virtual_2.y(1) = hy_2_hat(0); virtual_2.theta(1) = 0;
virtual_3.x(1) = hx_3_hat(0);virtual_3.y(1) = hy_3_hat(0); virtual_3.theta(1) = 0;
virtual_4.x(1) = hx_4_hat(0);virtual_4.y(1) = hy_4_hat(0); virtual_4.theta(1) = 0;
% 保存第一个时刻的编队偏移，用于后续只对“编队偏移”求导。
hx_1_arr(1) = hx_1_hat(0); hy_1_arr(1) = hy_1_hat(0);
hx_2_arr(1) = hx_2_hat(0); hy_2_arr(1) = hy_2_hat(0);
hx_3_arr(1) = hx_3_hat(0); hy_3_arr(1) = hy_3_hat(0);
hx_4_arr(1) = hx_4_hat(0); hy_4_arr(1) = hy_4_hat(0);
% 跟随者初始位置与角朝向
follower_1.x(1) = virtual_1.x(1); follower_1.y(1) = virtual_1.y(1); follower_1.theta(1) = virtual_1.theta(1);
follower_2.x(1) = virtual_2.x(1); follower_2.y(1) = virtual_2.y(1); follower_2.theta(1) = virtual_2.theta(1);
follower_3.x(1) = virtual_3.x(1); follower_3.y(1) = virtual_3.y(1); follower_3.theta(1) = virtual_3.theta(1);
follower_4.x(1) = virtual_4.x(1); follower_4.y(1) = virtual_4.y(1); follower_4.theta(1) = virtual_4.theta(1);
% 控制器参数初始值赋值
k1_arr_f1(1) = 1; k2_arr_f1(1) = 50; k3_arr_f1(1) = 1; k4_arr_f1(1) = 0.16;
k1_arr_f2(1) = 1; k2_arr_f2(1) = 50; k3_arr_f2(1) = 1; k4_arr_f2(1) = 0.16;
k1_arr_f3(1) = 1; k2_arr_f3(1) = 50; k3_arr_f3(1) = 1; k4_arr_f3(1) = 0.16;
k1_arr_f4(1) = 1; k2_arr_f4(1) = 50; k3_arr_f4(1) = 1; k4_arr_f4(1) = 0.16;
% 因为虚拟载具的位置计算是落后一个时钟刻度的，因此循环的起始位置时刻是2，所以这里需要再给follower的初始位置额外加一个赋值
    [follower_1.x(2), follower_1.y(2), follower_1.theta(2)] = ...
        update_agent_state(follower_1.x(1), follower_1.y(1), follower_1.theta(1), follower_1.v(1), follower_1.w(1), dt);
    [follower_2.x(2), follower_2.y(2), follower_2.theta(2)] = ...
        update_agent_state(follower_2.x(1), follower_2.y(1), follower_2.theta(1), follower_2.v(1), follower_2.w(1), dt);
    [follower_3.x(2), follower_3.y(2), follower_3.theta(2)] = ...
        update_agent_state(follower_3.x(1), follower_3.y(1), follower_3.theta(1), follower_3.v(1), follower_3.w(1), dt);
    [follower_4.x(2), follower_4.y(2), follower_4.theta(2)] = ...
        update_agent_state(follower_4.x(1), follower_4.y(1), follower_4.theta(1), follower_4.v(1), follower_4.w(1), dt);

%% 主仿真循环
for k = 2:N
    current_t = t(k);
 
    % 给领导者赋值速度
    leader.v(k) = v0(current_t);
    leader.v(k) = clamp_value_to_limits_helper(leader.v(k), v_min, v_max);
    leader.w(k) = w0(current_t);
    leader.w(k) = clamp_value_to_limits_helper(leader.w(k), w_min, w_max);

    % Stage 01：根据真实位置只生成一次公共带噪声观测。
    % 四个跟随者共享这两个数值，绝不分别抽取四组噪声。
    [leader_observation.x(k), leader_observation.y(k)] = ...
        stage_01_generate_noisy_leader_observation( ...
        leader.x(k), leader.y(k), injected_position_noise_std_x, injected_position_noise_std_y);

    % Stage 02：用直接传递的 theta、w 和上一后验 [x;y;v] 预测当前先验。
    [prior_state_k, prior_covariance_k] = stage_02_predict_leader_probability_distribution( ...
        leader_posterior.state(:,k-1), leader_posterior.covariance(:,:,k-1), ...
        leader.theta(k-1), leader.w(k-1), dt, process_acceleration_std);
    leader_prior = store_leader_estimate_fields(leader_prior, k, ...
        prior_state_k, prior_covariance_k, leader.theta(k), leader.w(k));

    % Stage 03：一次性融合二维位置观测，速度 v 由交叉协方差递归修正。
    [posterior_state_k, posterior_covariance_k, kalman_gain_k] = ...
        stage_03_fuse_leader_observation_distribution( ...
        prior_state_k, prior_covariance_k, ...
        [leader_observation.x(k); leader_observation.y(k)], ...
        kalman_assumed_observation_std_x^2, kalman_assumed_observation_std_y^2);
    leader_posterior = store_leader_estimate_fields(leader_posterior, k, ...
        posterior_state_k, posterior_covariance_k, leader.theta(k), leader.w(k));
    leader_posterior.kalman_gain(:,:,k) = kalman_gain_k;
    leader_posterior.kalman_gain_x(:,k) = [kalman_gain_k(1,1); cos(leader.theta(k))*kalman_gain_k(3,1)];
    leader_posterior.kalman_gain_y(:,k) = [kalman_gain_k(2,2); sin(leader.theta(k))*kalman_gain_k(3,2)];

    % Stage 04：同一 [x;y;v] 模型向前传播 0.1 s，theta、w 只作已知输入。
    [preview_state_k, preview_covariance_k] = stage_04_predict_leader_position_ahead( ...
        posterior_state_k, posterior_covariance_k, leader.theta(k), leader.w(k), ...
        preview_horizon, process_acceleration_std);
    preview_theta_k = wrap_angle_to_pi_helper(leader.theta(k) + leader.w(k) * preview_horizon);
    leader_preview = store_leader_estimate_fields(leader_preview, k, ...
        preview_state_k, preview_covariance_k, preview_theta_k, leader.w(k));
    leader_preview.variance_x(k) = preview_covariance_k(1,1);
    leader_preview.variance_y(k) = preview_covariance_k(2,2);

    % 默认使用前视领导者；关闭前视控制时退回当前后验领导者，而不是使用真实位置。
    if preview_used_for_control
        leader_reference_x = leader_preview.x(k);
        leader_reference_y = leader_preview.y(k);
        leader_reference_velocity_x = leader_preview.vx(k);
        leader_reference_velocity_y = leader_preview.vy(k);
    else
        leader_reference_x = leader_posterior.x(k);
        leader_reference_y = leader_posterior.y(k);
        leader_reference_velocity_x = leader_posterior.vx(k);
        leader_reference_velocity_y = leader_posterior.vy(k);
    end
    % 位置滤波不负责估计停车时不可观测的航向；编队旋转继续使用领导者航向状态。
    % 实车中该量应来自航向传感器/里程计，而不是对近零速度向量做 atan2。
    leader_reference_theta = leader.theta(k);
    leader_reference_theta_previous = leader.theta(k-1);

    % 确认编队的期望距离变化如代码开头所示
    hx_1_arr(k) = hx_1_hat(current_t);
    hy_1_arr(k) = hy_1_hat(current_t);
    hx_2_arr(k) = hx_2_hat(current_t);
    hy_2_arr(k) = hy_2_hat(current_t);
    hx_3_arr(k) = hx_3_hat(current_t);
    hy_3_arr(k) = hy_3_hat(current_t);
    hx_4_arr(k) = hx_4_hat(current_t);
    hy_4_arr(k) = hy_4_hat(current_t);

    % Stage 05：根据滤波后的领导者参考生成四个编队参考载具。
    % 这些参考载具参与控制，但按最终显示方案不在左侧动画中绘制。
    [virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), virtual_1.v(k-1), virtual_1.w(k-1), offset_velocity_x_1, offset_velocity_y_1] = ...
    stage_05_generate_formation_reference_agent(hx_1_arr(k), hy_1_arr(k), hx_1_arr(k-1), hy_1_arr(k-1), leader_reference_x, leader_reference_y, leader_reference_theta, leader_reference_theta_previous, leader_reference_velocity_x, leader_reference_velocity_y, virtual_1.theta(k-1), dt);
    [virtual_2.x(k), virtual_2.y(k), virtual_2.theta(k), virtual_2.v(k-1), virtual_2.w(k-1), offset_velocity_x_2, offset_velocity_y_2] = ...
    stage_05_generate_formation_reference_agent(hx_2_arr(k), hy_2_arr(k), hx_2_arr(k-1), hy_2_arr(k-1), leader_reference_x, leader_reference_y, leader_reference_theta, leader_reference_theta_previous, leader_reference_velocity_x, leader_reference_velocity_y, virtual_2.theta(k-1), dt);
    [virtual_3.x(k), virtual_3.y(k), virtual_3.theta(k), virtual_3.v(k-1), virtual_3.w(k-1), offset_velocity_x_3, offset_velocity_y_3] = ...
    stage_05_generate_formation_reference_agent(hx_3_arr(k), hy_3_arr(k), hx_3_arr(k-1), hy_3_arr(k-1), leader_reference_x, leader_reference_y, leader_reference_theta, leader_reference_theta_previous, leader_reference_velocity_x, leader_reference_velocity_y, virtual_3.theta(k-1), dt);
    [virtual_4.x(k), virtual_4.y(k), virtual_4.theta(k), virtual_4.v(k-1), virtual_4.w(k-1), offset_velocity_x_4, offset_velocity_y_4] = ...
    stage_05_generate_formation_reference_agent(hx_4_arr(k), hy_4_arr(k), hx_4_arr(k-1), hy_4_arr(k-1), leader_reference_x, leader_reference_y, leader_reference_theta, leader_reference_theta_previous, leader_reference_velocity_x, leader_reference_velocity_y, virtual_4.theta(k-1), dt);

    virtual_offset_velocity(:,1,k) = [offset_velocity_x_1; offset_velocity_y_1; hypot(offset_velocity_x_1, offset_velocity_y_1)];
    virtual_offset_velocity(:,2,k) = [offset_velocity_x_2; offset_velocity_y_2; hypot(offset_velocity_x_2, offset_velocity_y_2)];
    virtual_offset_velocity(:,3,k) = [offset_velocity_x_3; offset_velocity_y_3; hypot(offset_velocity_x_3, offset_velocity_y_3)];
    virtual_offset_velocity(:,4,k) = [offset_velocity_x_4; offset_velocity_y_4; hypot(offset_velocity_x_4, offset_velocity_y_4)];

    % function [vir_x_k, vir_y_k, vir_theta_k, vir_v_km, vir_w_km] = ...
    % stage_05_generate_formation_reference_agent(...)

    % 计算跟随者与虚拟载具的追踪误差，并将 误差的输出 转换到虚拟载具局部坐标系下
    % 时间轴：这里计算的是k时刻\当下的误差
    [Dx_1, Dy_1, Dtheta_1] = compute_tracking_error(virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), follower_1.x(k), follower_1.y(k), follower_1.theta(k));
    [Dx_2, Dy_2, Dtheta_2] = compute_tracking_error(virtual_2.x(k), virtual_2.y(k), virtual_2.theta(k), follower_2.x(k), follower_2.y(k), follower_2.theta(k));
    [Dx_3, Dy_3, Dtheta_3] = compute_tracking_error(virtual_3.x(k), virtual_3.y(k), virtual_3.theta(k), follower_3.x(k), follower_3.y(k), follower_3.theta(k));
    [Dx_4, Dy_4, Dtheta_4] = compute_tracking_error(virtual_4.x(k), virtual_4.y(k), virtual_4.theta(k), follower_4.x(k), follower_4.y(k), follower_4.theta(k));
    % function [Dx_vp, Dy_vp, Dtheta_vp] = compute_tracking_error(virtual_x, virtual_y, virtual_theta, follower_x, follower_y, follower_theta)
    error_1(:,k) = [Dx_1; Dy_1; Dtheta_1];
    error_2(:,k) = [Dx_2; Dy_2; Dtheta_2];
    error_3(:,k) = [Dx_3; Dy_3; Dtheta_3];
    error_4(:,k) = [Dx_4; Dy_4; Dtheta_4];

    % TVF控制器-增益系数数更新
    [k1_arr_f1(k), k2_arr_f1(k), k3_arr_f1(k), k4_arr_f1(k)] = ...
        update_control_gains(virtual_1.v(k-1), v_max, w_max);
    [k1_arr_f2(k), k2_arr_f2(k), k3_arr_f2(k), k4_arr_f2(k)] = ...
        update_control_gains(virtual_2.v(k-1), v_max, w_max);
    [k1_arr_f3(k), k2_arr_f3(k), k3_arr_f3(k), k4_arr_f3(k)] = ...
        update_control_gains(virtual_3.v(k-1), v_max, w_max);
    [k1_arr_f4(k), k2_arr_f4(k), k3_arr_f4(k), k4_arr_f4(k)] = ...
        update_control_gains(virtual_4.v(k-1), v_max, w_max);
    % function [k1, k2, k3, k4] = ...
    %     update_control_gains(virtual_v, v_max, w_max)

    % Stage 06：使用 ver2 TVF 控制律计算四个跟随者的线速度和角速度。
    [follower_1.v(k), follower_1.w(k)] = ...
    stage_06_calculate_tvf_tracking_control(virtual_1.v(k-1), Dtheta_1,...
                k1_arr_f1(k), k2_arr_f1(k), k3_arr_f1(k), k4_arr_f1(k),...
                Dx_1, Dy_1, v_max, w_max);
    [follower_2.v(k), follower_2.w(k)] = ...
    stage_06_calculate_tvf_tracking_control(virtual_2.v(k-1), Dtheta_2,...
                k1_arr_f2(k), k2_arr_f2(k), k3_arr_f2(k), k4_arr_f2(k),...
                Dx_2, Dy_2, v_max, w_max);
    [follower_3.v(k), follower_3.w(k)] = ...
    stage_06_calculate_tvf_tracking_control(virtual_3.v(k-1), Dtheta_3,...
                k1_arr_f3(k), k2_arr_f3(k), k3_arr_f3(k), k4_arr_f3(k),...
                Dx_3, Dy_3, v_max, w_max);
    [follower_4.v(k), follower_4.w(k)] = ...
    stage_06_calculate_tvf_tracking_control(virtual_4.v(k-1), Dtheta_4,...
                k1_arr_f4(k), k2_arr_f4(k), k3_arr_f4(k), k4_arr_f4(k),...
                Dx_4, Dy_4, v_max, w_max);
    % function [follower_p.v, follower_p.w] = ...
    % TVF_control_law(virtual_theta, virtual_v, follower_theta, ...
    %             k1, k2, k3, k4, ...
    %             Dx_vp, Dy_vp, v_max, w_max)

    % 用于验证数据传输正常的测试算法
    % [follower_1.v(k), follower_1.w(k)] = tracking_test_control_law(Dx_1, Dy_1, Dtheta_1, v_max, w_max);

    % 更新所有实际载具的状态
    [leader.x(k+1), leader.y(k+1), leader.theta(k+1)] = ...
        update_agent_state(leader.x(k), leader.y(k), leader.theta(k), leader.v(k), leader.w(k), dt);
    [follower_1.x(k+1), follower_1.y(k+1), follower_1.theta(k+1)] = ...
        update_agent_state(follower_1.x(k), follower_1.y(k), follower_1.theta(k), follower_1.v(k), follower_1.w(k), dt);
    [follower_2.x(k+1), follower_2.y(k+1), follower_2.theta(k+1)] = ...
        update_agent_state(follower_2.x(k), follower_2.y(k), follower_2.theta(k), follower_2.v(k), follower_2.w(k), dt);
    [follower_3.x(k+1), follower_3.y(k+1), follower_3.theta(k+1)] = ...
        update_agent_state(follower_3.x(k), follower_3.y(k), follower_3.theta(k), follower_3.v(k), follower_3.w(k), dt);
    [follower_4.x(k+1), follower_4.y(k+1), follower_4.theta(k+1)] = ...
        update_agent_state(follower_4.x(k), follower_4.y(k), follower_4.theta(k), follower_4.v(k), follower_4.w(k), dt);
    % function [agent_x_kp, agent_y_kp, agent_theta_kp] = update_agent_state(agent_x_k, agent_y_k, agent_theta_k, agent_v_k, agent_w_k, dt)
end

% 最后时刻赋值
virtual_1.x(N) = virtual_1.x(N-1); virtual_1.y(N) = virtual_1.y(N-1); virtual_1.theta(N) = virtual_1.theta(N-1);
virtual_2.x(N) = virtual_2.x(N-1); virtual_2.y(N) = virtual_2.y(N-1); virtual_2.theta(N) = virtual_2.theta(N-1);
virtual_3.x(N) = virtual_3.x(N-1); virtual_3.y(N) = virtual_3.y(N-1); virtual_3.theta(N) = virtual_3.theta(N-1);
virtual_4.x(N) = virtual_4.x(N-1); virtual_4.y(N) = virtual_4.y(N-1); virtual_4.theta(N) = virtual_4.theta(N-1);

error_magnitude_1 = sqrt(error_1(1,:).^2 + error_1(2,:).^2);
error_magnitude_2 = sqrt(error_2(1,:).^2 + error_2(2,:).^2);
error_magnitude_3 = sqrt(error_3(1,:).^2 + error_3(2,:).^2);
error_magnitude_4 = sqrt(error_4(1,:).^2 + error_4(2,:).^2);

fprintf('\n=== 四个 virtual 的 offset 速度诊断 ===\n');
for virtual_index = 1:4
    offset_speed_history = squeeze(virtual_offset_velocity(3,virtual_index,:));
    [maximum_offset_speed, maximum_offset_index] = max(offset_speed_history);
    fprintf('virtual_%d: 最大 offset 速度 %.6f m/s，发生在 t=%.3f s\n', ...
        virtual_index, maximum_offset_speed, t(maximum_offset_index));
end

% 保存完整仿真和概率融合数据；动画由独立程序读取本 MAT 文件生成。
save(mat_filename, 't', 'dt', 'leader', 'leader_observation', 'leader_prior', ...
    'leader_posterior', 'leader_preview', 'virtual_offset_velocity', 'virtual_1', 'virtual_2', ...
    'virtual_3', 'virtual_4', 'follower_1', 'follower_2', 'follower_3', ...
    'follower_4', 'error_1', 'error_2', 'error_3', 'error_4', ...
    'preview_horizon', 'preview_used_for_control', ...
    'injected_position_noise_std_x', 'injected_position_noise_std_y', ...
    'kalman_assumed_observation_std_x', 'kalman_assumed_observation_std_y', ...
    'process_acceleration_std', ...
    'probability_display_axis');
fprintf('仿真数据已保存为 %s\n', mat_filename);
%% 06 Stage 01：生成唯一的公共带噪声领导者观测
function [observed_x, observed_y] = ...
    stage_01_generate_noisy_leader_observation( ...
    true_x, true_y, injected_position_noise_std_x, injected_position_noise_std_y)
% 输入 true_x/true_y 为仿真真实位置；输出是本时刻所有载具共享的观测。

observed_x = true_x + injected_position_noise_std_x * randn();
observed_y = true_y + injected_position_noise_std_y * randn();
end

%% 07 Stage 02：手写单一 [x;y;v] 模型的先验预测
function [prior_state, prior_covariance] = ...
    stage_02_predict_leader_probability_distribution( ...
    previous_posterior_state, previous_posterior_covariance, ...
    heading_angle, angular_velocity, prediction_interval, process_acceleration_std)
% theta 和 w 是外部直接传递的确定性输入，不进入 Kalman 状态和协方差。
% 对恒定 v、w 的非完整运动学在本时间段内作解析积分，避免把 v 拆成两个独立状态。

if abs(angular_velocity) > 1e-9
    displacement_coefficient_x = ...
        (sin(heading_angle + angular_velocity * prediction_interval) - sin(heading_angle)) / angular_velocity;
    displacement_coefficient_y = ...
        (cos(heading_angle) - cos(heading_angle + angular_velocity * prediction_interval)) / angular_velocity;
else
    displacement_coefficient_x = cos(heading_angle) * prediction_interval;
    displacement_coefficient_y = sin(heading_angle) * prediction_interval;
end

prior_state = [previous_posterior_state(1) + previous_posterior_state(3) * displacement_coefficient_x; ...
    previous_posterior_state(2) + previous_posterior_state(3) * displacement_coefficient_y; ...
    previous_posterior_state(3)];

state_transition = [1, 0, displacement_coefficient_x; ...
                    0, 1, displacement_coefficient_y; ...
                    0, 0, 1];
process_noise_mapping = [0.5 * displacement_coefficient_x * prediction_interval; ...
                         0.5 * displacement_coefficient_y * prediction_interval; ...
                         prediction_interval];
process_noise_covariance = process_acceleration_std^2 * ...
    (process_noise_mapping * process_noise_mapping.');
prior_covariance = state_transition * previous_posterior_covariance * ...
    state_transition.' + process_noise_covariance;
prior_covariance = 0.5 * (prior_covariance + prior_covariance.');
% 对照实验：忽略 x 与 y 预测误差之间的交叉协方差。
prior_covariance(1,2) = 0;
prior_covariance(2,1) = 0;
for diagonal_index = 1:3
    prior_covariance(diagonal_index, diagonal_index) = ensure_positive_variance_helper( ...
        prior_covariance(diagonal_index, diagonal_index), 1e-12);
end
end

%% 08 Stage 03：手写二维位置观测与 [x;y;v] 先验的高斯融合
function [posterior_state, posterior_covariance, kalman_gain] = ...
    stage_03_fuse_leader_observation_distribution( ...
    prior_state, prior_covariance, position_observation, ...
    observation_variance_x, observation_variance_y)
% H=[1 0 0; 0 1 0]；只测量 x、y，v 通过位置-速度交叉协方差递归修正。
% 2×2 创新协方差的逆按行列式显式计算，不调用 inv 或 Kalman 工具箱。

observation_variance_x = ensure_positive_variance_helper(observation_variance_x, 1e-12);
observation_variance_y = ensure_positive_variance_helper(observation_variance_y, 1e-12);
observation_covariance = [observation_variance_x, 0; 0, observation_variance_y];
innovation = position_observation - prior_state(1:2);
innovation_covariance = prior_covariance(1:2,1:2) + observation_covariance;
innovation_determinant = innovation_covariance(1,1) * innovation_covariance(2,2) - ...
    innovation_covariance(1,2) * innovation_covariance(2,1);
innovation_determinant = ensure_positive_variance_helper(innovation_determinant, 1e-18);
inverse_innovation_covariance = [innovation_covariance(2,2), -innovation_covariance(1,2); ...
    -innovation_covariance(2,1), innovation_covariance(1,1)] / innovation_determinant;

kalman_gain = prior_covariance(:,1:2) * inverse_innovation_covariance;
posterior_state = prior_state + kalman_gain * innovation;

measurement_matrix = [1, 0, 0; 0, 1, 0];
identity_minus_gain_times_measurement = eye(3) - kalman_gain * measurement_matrix;
posterior_covariance = identity_minus_gain_times_measurement * prior_covariance * ...
    identity_minus_gain_times_measurement.' + ...
    kalman_gain * observation_covariance * kalman_gain.';
posterior_covariance = 0.5 * (posterior_covariance + posterior_covariance.');
% 对照实验：保证传入下一次预测的后验 Pxy 仍为零。
posterior_covariance(1,2) = 0;
posterior_covariance(2,1) = 0;
for diagonal_index = 1:3
    posterior_covariance(diagonal_index, diagonal_index) = ensure_positive_variance_helper( ...
        posterior_covariance(diagonal_index, diagonal_index), 1e-12);
end
end

%% 09 Stage 04：由当前后验向前传播指定时间
function [preview_state, preview_covariance] = ...
    stage_04_predict_leader_position_ahead( ...
    current_posterior_state, current_posterior_covariance, ...
    heading_angle, angular_velocity, preview_horizon, process_acceleration_std)
% 前视值只用于可选控制/显示，不回灌到当前时刻的 Stage 03。

[preview_state, preview_covariance] = ...
    stage_02_predict_leader_probability_distribution( ...
    current_posterior_state, current_posterior_covariance, ...
    heading_angle, angular_velocity, preview_horizon, process_acceleration_std);
end

%% 10 领导者估计数据结构与兼容字段
function estimate = define_leader_estimate_state(N)
estimate.state = zeros(3,N);                 % 真正参与递推的 [x;y;v]
estimate.covariance = zeros(3,3,N);
estimate.x = zeros(1,N);
estimate.y = zeros(1,N);
estimate.vx = zeros(1,N);
estimate.vy = zeros(1,N);
estimate.v = zeros(1,N);                     % 有符号车体纵向速度
estimate.v_abs = zeros(1,N);                 % 速度模长诊断量
estimate.v_lateral = zeros(1,N);             % 车体横向速度诊断量
estimate.theta = zeros(1,N);                 % 直接传递，不滤波
estimate.w = zeros(1,N);                     % 直接传递，不滤波
estimate.state_x = zeros(2,N);               % 兼容旧动画：[x;vx]
estimate.state_y = zeros(2,N);               % 兼容旧动画：[y;vy]
estimate.covariance_x = zeros(2,2,N);
estimate.covariance_y = zeros(2,2,N);
end

function estimate = store_leader_estimate_fields(estimate, k, state, covariance, heading_angle, angular_velocity)
estimate.state(:,k) = state;
estimate.covariance(:,:,k) = covariance;
estimate.x(k) = state(1);
estimate.y(k) = state(2);
estimate.v(k) = state(3);
estimate.vx(k) = state(3) * cos(heading_angle);
estimate.vy(k) = state(3) * sin(heading_angle);
estimate.v_abs(k) = abs(state(3));
estimate.v_lateral(k) = -estimate.vx(k) * sin(heading_angle) + estimate.vy(k) * cos(heading_angle);
estimate.theta(k) = heading_angle;
estimate.w(k) = angular_velocity;

% 以下字段仅供旧动画和已有 MAT 数据读取逻辑兼容，不是两套独立 Kalman 状态。
estimate.state_x(:,k) = [estimate.x(k); estimate.vx(k)];
estimate.state_y(:,k) = [estimate.y(k); estimate.vy(k)];
jacobian_x = [1, 0, 0; 0, 0, cos(heading_angle)];
jacobian_y = [0, 1, 0; 0, 0, sin(heading_angle)];
estimate.covariance_x(:,:,k) = jacobian_x * covariance * jacobian_x.';
estimate.covariance_y(:,:,k) = jacobian_y * covariance * jacobian_y.';
end

%{
旧笛卡尔双滤波方案（仅作设计注释，不执行）：
    state_x = [x; vx]; state_y = [y; vy];
    分别以 [1 dt;0 1] 预测并独立融合 x、y 观测，再由 hypot(vx,vy) 合成速度。
该方案在低速时会使速度方向受噪声支配，也不能天然表达非完整载具只有一个纵向速度 v，
因此当前实现统一改为 [x;y;v]，theta、w 作为直接输入。
%}

% 代码辅助函数：绘制机器人的运动轨迹
function plot_robot_orientation(x, y, theta, color, linewidth)
    arrow_length = 1.5;
    dx = arrow_length * cos(theta);
    dy = arrow_length * sin(theta);
    plot([x, x+dx], [y, y+dy], color, 'LineWidth', linewidth);
end





%% 多智能体-幽灵载具追踪算法通用函数
% 多智能体辅助函数1：单个智能体状态结构体定义
function agent_state = define_agent_state(N)
    % 定义单个智能体的状态结构
    % 输入参数:
    %   N - 时间步数
    % 输出:
    %   agent_state - 包含所有状态变量的结构体
    
    agent_state = struct();
    agent_state.x = zeros(1, N);      % x坐标
    agent_state.y = zeros(1, N);      % y坐标
    agent_state.theta = zeros(1, N);  % 航向角
    agent_state.v = zeros(1, N);      % 线速度
    agent_state.w = zeros(1, N);      % 角速度
end

% 多智能体辅助函数2：更新智能体状态
% 注意时间轴： km means k minus 1,kp means k plus 1
function [agent_x_kp, agent_y_kp, agent_theta_kp] = update_agent_state(agent_x_k, agent_y_k, agent_theta_k, agent_v_k, agent_w_k, dt)
    agent_x_kp = agent_x_k + agent_v_k * cos(agent_theta_k) * dt;
    agent_y_kp = agent_y_k + agent_v_k * sin(agent_theta_k) * dt;
    agent_theta_kp = agent_theta_k + agent_w_k * dt;
    agent_theta_kp = wrap_angle_to_pi_helper(agent_theta_kp); % 角度区间回正
end

% Stage 05：计算 k 时刻编队参考位置，并形成 k-1 到 k 区间使用的参考速度。
% 平移速度直接取卡尔曼后验速度；只有确定性的编队旋转偏移采用相邻时刻差分。
% km means k minus 1，输出 virtual.v(k-1) 和 virtual.w(k-1) 以保持原时序。
function [vir_x_k, vir_y_k, vir_theta_k, vir_v_km, vir_w_km, ...
    offset_velocity_x, offset_velocity_y] = ...
    stage_05_generate_formation_reference_agent( ...
    hx_hat_k, hy_hat_k, hx_hat_km, hy_hat_km, ...
    leader_x_k, leader_y_k, leader_theta_k, leader_theta_km, ...
    leader_velocity_x_k, leader_velocity_y_k, vir_theta_km, dt)
    % 当前编队偏移由当前后验领导者航向和预设编队函数得到。
    offset_world_x_k = cos(leader_theta_k) * hx_hat_k - sin(leader_theta_k) * hy_hat_k;
    offset_world_y_k = sin(leader_theta_k) * hx_hat_k + cos(leader_theta_k) * hy_hat_k;
    vir_x_k = leader_x_k + offset_world_x_k;
    vir_y_k = leader_y_k + offset_world_y_k;

    % 只对编队偏移求导；领导者平移速度直接取卡尔曼后验速度，禁止再差分含噪位置。
    offset_world_x_km = cos(leader_theta_km) * hx_hat_km - sin(leader_theta_km) * hy_hat_km;
    offset_world_y_km = sin(leader_theta_km) * hx_hat_km + cos(leader_theta_km) * hy_hat_km;
    offset_velocity_x = (offset_world_x_k - offset_world_x_km) / dt;
    offset_velocity_y = (offset_world_y_k - offset_world_y_km) / dt;
    vir_velocity_x = leader_velocity_x_k + offset_velocity_x;
    vir_velocity_y = leader_velocity_y_k + offset_velocity_y;

    vir_v_km = hypot(vir_velocity_x, vir_velocity_y);
    if vir_v_km > 1e-9
        vir_theta_k = atan2(vir_velocity_y, vir_velocity_x);
    else
        vir_theta_k = vir_theta_km;
    end
    % 航向差除以 dt 得到 rad/s；若用该角速度推进角度，状态方程必须再乘 dt。
    vir_w_km = wrap_angle_to_pi_helper(vir_theta_k - vir_theta_km) / dt;
end

% 多智能体辅助函数5：计算追踪误差
function [Dx_vp, Dy_vp, Dtheta_vp] = compute_tracking_error(virtual_x, virtual_y, virtual_theta, follower_x, follower_y, follower_theta)
    % 计算世界坐标系下的误差
    Delta_x_vp_world = virtual_x - follower_x;
    Delta_y_vp_world = virtual_y - follower_y;
    % 计算幽灵载具局部坐标系 下的位置误差
    Dx_vp = cos(follower_theta) * Delta_x_vp_world + sin(follower_theta) * Delta_y_vp_world;
    Dy_vp = -sin(follower_theta) * Delta_x_vp_world + cos(follower_theta) * Delta_y_vp_world;
    Dtheta_vp = virtual_theta - follower_theta;
    Dtheta_vp = wrap_angle_to_pi_helper(Dtheta_vp);
end

% 多智能体辅助函数6：追踪效果测试算法
function [follower_v, follower_w] = ...
    tracking_test_control_law(Dx_vp, Dy_vp, Dtheta_vp, v_max, w_max)
    % 简单比例控制律
    kp_pos = 1; kp_angle = 1;
    vp = kp_pos * sqrt(Dx_vp^2 + Dy_vp^2);
    wp = kp_angle * Dtheta_vp;
    follower_v = clamp_value_to_limits_helper(vp, 0, v_max);
    follower_w = clamp_value_to_limits_helper(wp, -w_max, w_max);
end

%% TVF控制逻辑
% 辅助函数A：TVF控制律
% 时间轴问题：根据上一时刻传回的virtual_v和follower_theta计算当前时刻的控制输入
function [follower_v, follower_w] = ...
stage_06_calculate_tvf_tracking_control(virtual_v, Dtheta_vp, ...
                k1, k2, k3, k4, ...
                Dx_vp, Dy_vp, v_max, w_max)
    % 控制律
    D_p = max(v_max, 1) + max(w_max, 1) * sqrt(Dx_vp^2 + Dy_vp^2);
    s_p = (k4 * Dy_vp / D_p) + Dtheta_vp;
    %vp = virtual_v;
    vp = virtual_v + k1 * Dx_vp / sqrt(Dx_vp^2 + Dy_vp^2 + k2);
    %wp = Dtheta_vp;
    wp = k3 * sign(s_p);
    follower_v = clamp_value_to_limits_helper(vp, -v_max, v_max);
    follower_w = clamp_value_to_limits_helper(wp, -w_max, w_max);
    % 如果是利用舵机进行控制，运行下面的语句，通过转弯半径直接计算舵机的角朝向
    % turning_radius = follower_v / (follower_w + 1e-6); % 防止除零
    % 
end

% 辅助函数B：校正系数更新
function [k1, k2, k3, k4] = ...
    update_control_gains(virtual_v, v_max, w_max)
    % 控制系数更新
      k1 = max(v_max - virtual_v, virtual_v - v_max);
      k2 = v_max / 2;
      k3 = w_max; %根据舵机的最大扭矩在实际硬件中的表现确定，应该远比1要大，这里取较小的值，确保模拟收敛的情况下实物一定收敛
      k4 = min(4 * k1 / v_max, min(2 * (k3 - w_max) / 3, max(w_max, 1) * pi / 2)); 
end





% 代码本体的改进逻辑
% 1.最大抛射速度，确保虚拟载具的速度不会超过物理载具的最大速度
%   我们之前的函数是直接根据载具的物理性能来直接定义所有载具的速度（包括leader和virtual）
%   但实际上，虚拟载具的速度不应超过物理载具的最大速度，否则物理载具无法追踪
%   这里应当增加一个函数来限制ledaer的最大速度，通过计算最大编队距离+最大角速度变化时带来的速度增量，用v_max减去这个值来限制leader的速度
% 2.虚拟载具的角速度计算改进，使用曲率计算
%   修改辅助函数4中vir_w_km的计算方式，使用曲率公式来计算角速度，而不是直接求导
% 3.控制器参数的自适应更新（完成
% 4.时间轴逻辑问题修正，确保每个变量的计算基于正确的时间步骤（完成
% 5.在编写新算法时如小车的追踪效果有问题，尝试运行测试算法，确保基础代码的正确性

% 论文的撰写思路
% 1.引言部分，介绍非完整约束车辆编队控制的背景和意义
% 2.相关工作，综述现有的编队控制方法及其局限性
% 3.问题描述，明确系统模型和控制目标
% 4.TVF控制方法，详细介绍时变编队控制律的设计和理论分析
% 5.仿真结果，展示仿真设置、结果和性能分析
% 6.结论与未来工作，总结研究成果并提出未来的研究方向
% 7.参考文献，列出所有引用的文献资料
% 8.附录（可选），提供详细的数学推导或额外的仿真数据

% 只要设计不同的4和5部分，其他部分可以沿用之前的论文框架
% 简而言之，我们以virtual载具的速度作为追踪信号，每重新设计一个控制器并且做一篇仿真，就会有一篇新论文
% 当然了别傻不拉几的每一篇的1-3部分都一样，这样会被审稿人鄙视的

