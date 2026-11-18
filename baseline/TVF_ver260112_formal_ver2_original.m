%% nonholonomic vehicle formation tracking using time-varying formation (TVF) control
%% reformat the code from the source version code into a formal version
%% so you can more easilier add other agents into the simulation

%% 为了更好的表现论文中TVF算法的追踪效果，这里把follower的初始位置放在原本的desired position附近

clear; close all; clc;

%%是否生成收敛效果图和动画
convergence_performance = 0;
generate_animation = ~convergence_performance;

%%分别用于查看虚拟载具和真实载具的运行轨迹
follower_orbit = 1;
virtual_orbit = ~follower_orbit;

%每次生成的动画和收敛性对比图像都会在名字末尾加上生成的时间戳
timestamp = datestr(now, 'yymmdd-HHMMSS');
gif_filename = sprintf('formation_tracking_%s.gif', timestamp);
jpg_filename = sprintf('convergence_performance_%s.jpg', timestamp);

% 参数设置
dt = 0.01;          % 时间步长
T = 20;             % 总时间
t = 0:dt:T;         % 时间向量
N = length(t);

v_min = -50; v_max = 50;            % 线速度约束
w_min = -1; w_max = 1;            % 角速度约束

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















%% 初始状态赋值
% 领导者初始位置与角朝向
leader.x(1) = 0; leader.y(1) = 0; leader.theta(1) = 0; leader.v(1) = v0(0); leader.w(1) = w0(0);
% 幽灵载具初始位置
virtual_1.x(1) = hx_1_hat(0);virtual_1.y(1) = hy_1_hat(0); virtual_1.theta(1) = 0;
virtual_2.x(1) = hx_2_hat(0);virtual_2.y(1) = hy_2_hat(0); virtual_2.theta(1) = 0;
virtual_3.x(1) = hx_3_hat(0);virtual_3.y(1) = hy_3_hat(0); virtual_3.theta(1) = 0;
virtual_4.x(1) = hx_4_hat(0);virtual_4.y(1) = hy_4_hat(0); virtual_4.theta(1) = 0;
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
    leader.v(k) = max(v_min, min(v_max, leader.v(k))); % 速度约束
    leader.w(k) = w0(current_t);
    leader.w(k) = max(w_min, min(w_max, leader.w(k))); % 角速度约束

    % 确认编队的期望距离变化如代码开头所示
    hx_1_arr(k) = hx_1_hat(current_t);
    hy_1_arr(k) = hy_1_hat(current_t);
    hx_2_arr(k) = hx_2_hat(current_t);
    hy_2_arr(k) = hy_2_hat(current_t);
    hx_3_arr(k) = hx_3_hat(current_t);
    hy_3_arr(k) = hy_3_hat(current_t);
    hx_4_arr(k) = hx_4_hat(current_t);
    hy_4_arr(k) = hy_4_hat(current_t);

    % 计算虚拟机器人的agent状态
    [virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), virtual_1.v(k-1), virtual_1.w(k-1)] = ...
    compute_virtual_vehicle(hx_1_arr(k), hy_1_arr(k), leader.x(k), leader.y(k), leader.theta(k), virtual_1.x(k-1), virtual_1.y(k-1), virtual_1.theta(k-1), dt);
    [virtual_2.x(k), virtual_2.y(k), virtual_2.theta(k), virtual_2.v(k-1), virtual_2.w(k-1)] = ...
    compute_virtual_vehicle(hx_2_arr(k), hy_2_arr(k), leader.x(k), leader.y(k), leader.theta(k), virtual_2.x(k-1), virtual_2.y(k-1), virtual_2.theta(k-1), dt);
    [virtual_3.x(k), virtual_3.y(k), virtual_3.theta(k), virtual_3.v(k-1), virtual_3.w(k-1)] = ...
    compute_virtual_vehicle(hx_3_arr(k), hy_3_arr(k), leader.x(k), leader.y(k), leader.theta(k), virtual_3.x(k-1), virtual_3.y(k-1), virtual_3.theta(k-1), dt);
    [virtual_4.x(k), virtual_4.y(k), virtual_4.theta(k), virtual_4.v(k-1), virtual_4.w(k-1)] = ...
    compute_virtual_vehicle(hx_4_arr(k), hy_4_arr(k), leader.x(k), leader.y(k), leader.theta(k), virtual_4.x( k-1 ), virtual_4.y( k-1 ), virtual_4.theta( k-1 ), dt);

    % function [vir_x_k, vir_y_k, vir_theta_k, vir_v_km, vir_w_km] = ...
    % compute_virtual_vehicle(hx_hat_k, hy_hat_k, leader_x_k, leader_y_k, leader_theta_k, vir_x_km, vir_y_km, vir_theta_km)

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

    % TVF控制律计算载具的速度角速度
    [follower_1.v(k), follower_1.w(k)] = ...
    TVF_control_law(virtual_1.v(k-1), Dtheta_1,...
                k1_arr_f1(k), k2_arr_f1(k), k3_arr_f1(k), k4_arr_f1(k),...
                Dx_1, Dy_1, v_max, w_max);
    [follower_2.v(k), follower_2.w(k)] = ...
    TVF_control_law(virtual_2.v(k-1), Dtheta_2,...
                k1_arr_f2(k), k2_arr_f2(k), k3_arr_f2(k), k4_arr_f2(k),...
                Dx_2, Dy_2, v_max, w_max);
    [follower_3.v(k), follower_3.w(k)] = ...
    TVF_control_law(virtual_3.v(k-1), Dtheta_3,...
                k1_arr_f3(k), k2_arr_f3(k), k3_arr_f3(k), k4_arr_f3(k),...
                Dx_3, Dy_3, v_max, w_max);
    [follower_4.v(k), follower_4.w(k)] = ...
    TVF_control_law(virtual_4.v(k-1), Dtheta_4,...
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

if generate_animation == true

    %% 生成动画
    fprintf('生成动画...\n');
    figure('Position', [200, 200, 1000, 800]);

    % 选择动画帧（每10帧取一帧）
    frame_indices = 1:10:N;
    num_frames = length(frame_indices);

    if follower_orbit ==1
        for idx = 1:num_frames
            k = frame_indices(idx);

            clf;

            % 绘制轨迹
            plot(leader.x(1:k), leader.y(1:k), 'k-', 'LineWidth', 2); hold on;
            % plot(virtual_1.x(1:k), virtual_1.y(1:k), 'r--', 'LineWidth', 2);
            plot(follower_1.x(1:k), follower_1.y(1:k), 'b-', 'LineWidth', 1.5);
            plot(follower_2.x(1:k), follower_2.y(1:k), 'g-', 'LineWidth', 1.5);
            plot(follower_3.x(1:k), follower_3.y(1:k), 'm-', 'LineWidth', 1.5);
            plot(follower_4.x(1:k), follower_4.y(1:k), 'c-', 'LineWidth', 1.5);
            % plot(follower_1.x(1:k), follower_1.y(1:k), 'g-', 'LineWidth', 1.5);

            % 绘制当前位置
            scatter(leader.x(k), leader.y(k), 200, 'k', 'filled', 'd');
            %scatter(virtual_1.x(k), virtual_1.y(k), 150, 'r', 'filled');
            scatter(follower_1.x(k), follower_1.y(k), 100, 'b', 'filled');
            scatter(follower_2.x(k), follower_2.y(k), 100, 'g', 'filled');
            scatter(follower_3.x(k), follower_3.y(k), 100, 'm', 'filled');
            scatter(follower_4.x(k), follower_4.y(k), 100, 'c', 'filled');
            %scatter(follower_1.x(k), follower_1.y(k), 100, 'g', 'filled');

            % 绘制方向指示
            plot_robot_orientation(leader.x(k), leader.y(k), leader.theta(k), 'k', 2);
            %plot_robot_orientation(virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), 'r', 1.5);
            plot_robot_orientation(follower_1.x(k), follower_1.y(k), follower_1.theta(k), 'r', 1.5);
            plot_robot_orientation(follower_2.x(k), follower_2.y(k), follower_2.theta(k), 'r', 1.5);
            plot_robot_orientation(follower_3.x(k), follower_3.y(k), follower_3.theta(k), 'r', 1.5);
            plot_robot_orientation(follower_4.x(k), follower_4.y(k), follower_4.theta(k), 'r', 1.5);

            xlabel('X坐标'); ylabel('Y坐标');
            title(sprintf('运动轨迹 (t = %.1fs)', t(k)));
            legend('领导者', '跟随1', '跟随2', '跟随3', '跟随4', 'Location', 'best');
            grid on; axis equal;

            drawnow;

            % 保存帧（用于生成GIF）
            if idx == 1
                % 初始化GIF
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                imwrite(imind,cm, gif_filename,'gif','Loopcount',inf,'DelayTime',0.1);
            else
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                imwrite(imind,cm, gif_filename,'gif','WriteMode','append','DelayTime',0.1);
            end
        end
    end

    if virtual_orbit ==1
        for idx = 1:num_frames
            k = frame_indices(idx);

            clf;

            % 绘制轨迹
            plot(leader.x(1:k), leader.y(1:k), 'k-', 'LineWidth', 2); hold on;
            % plot(virtual_1.x(1:k), virtual_1.y(1:k), 'r--', 'LineWidth', 2);
            plot(virtual_1.x(1:k), virtual_1.y(1:k), 'b-', 'LineWidth', 1.5);
            plot(virtual_2.x(1:k), virtual_2.y(1:k), 'g-', 'LineWidth', 1.5);
            plot(virtual_3.x(1:k), virtual_3.y(1:k), 'm-', 'LineWidth', 1.5);
            plot(virtual_4.x(1:k), virtual_4.y(1:k), 'c-', 'LineWidth', 1.5);
            % plot(follower_1.x(1:k), follower_1.y(1:k), 'g-', 'LineWidth', 1.5);

            % 绘制当前位置
            scatter(leader.x(k), leader.y(k), 200, 'k', 'filled', 'd');
            %scatter(virtual_1.x(k), virtual_1.y(k), 150, 'r', 'filled');
            scatter(virtual_1.x(k), virtual_1.y(k), 100, 'b', 'filled');
            scatter(virtual_2.x(k), virtual_2.y(k), 100, 'g', 'filled');
            scatter(virtual_3.x(k), virtual_3.y(k), 100, 'm', 'filled');
            scatter(virtual_4.x(k), virtual_4.y(k), 100, 'c', 'filled');
            %scatter(follower_1.x(k), follower_1.y(k), 100, 'g', 'filled');

            % 绘制方向指示
            plot_robot_orientation(leader.x(k), leader.y(k), leader.theta(k), 'k', 2);
            %plot_robot_orientation(virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), 'r', 1.5);
            plot_robot_orientation(virtual_1.x(k), virtual_1.y(k), virtual_1.theta(k), 'r', 1.5);
            plot_robot_orientation(virtual_2.x(k), virtual_2.y(k), virtual_2.theta(k), 'r', 1.5);
            plot_robot_orientation(virtual_3.x(k), virtual_3.y(k), virtual_3.theta(k), 'r', 1.5);
            plot_robot_orientation(virtual_4.x(k), virtual_4.y(k), virtual_4.theta(k), 'r', 1.5);

            xlabel('X坐标'); ylabel('Y坐标');
            title(sprintf('运动轨迹 (t = %.1fs)', t(k)));
            legend('领导者', '幽灵1', '幽灵2', '幽灵3', '幽灵4', 'Location', 'best');
            grid on; axis equal;

            drawnow;

            % 保存帧（用于生成GIF）
            if idx == 1
                % 初始化GIF
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                imwrite(imind,cm, gif_filename,'gif','Loopcount',inf,'DelayTime',0.1);
            else
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                imwrite(imind,cm, gif_filename,'gif','WriteMode','append','DelayTime',0.1);
            end
        end
    end
  fprintf('动画已保存为 %s\n', gif_filename);
end

if convergence_performance == true

  %% 生成收敛效果图
  fprintf('生成收敛效果图...\n');
  figure('Position', [200, 200, 1200, 800]);

  subplot(3,1,1);
  plot(t, error_1(1,:), 'r-', 'LineWidth', 1.5); hold on;
  plot(t, error_2(1,:), 'g-', 'LineWidth', 1.5);
  plot(t, error_3(1,:), 'b-', 'LineWidth', 1.5);
  plot(t, error_4(1,:), 'm-', 'LineWidth', 1.5);
  xlabel('时间 (s)'); ylabel('位置误差 Dx_vp (m)');
  title('跟随者与虚拟载具的X位置误差');
  legend('跟随1', '跟随2', '跟随3', '跟随4');
  grid on;

  subplot(3,1,2);
  plot(t, error_1(2,:), 'r-', 'LineWidth', 1.5); hold on;
  plot(t, error_2(2,:), 'g-', 'LineWidth', 1.5);
  plot(t, error_3(2,:), 'b-', 'LineWidth', 1.5);
  plot(t, error_4(2,:), 'm-', 'LineWidth', 1.5);
  xlabel('时间 (s)'); ylabel('位置误差 Dy_vp (m)');
  title('跟随者与虚拟载具的Y位置误差');
  legend('跟随1', '跟随2', '跟随3', '跟随4');
  grid on;

  subplot(3,1,3);
  plot(t, error_magnitude_1(:), 'r-', 'LineWidth', 1.5); hold on;
  plot(t, error_magnitude_2(:), 'g-', 'LineWidth', 1.5);
  plot(t, error_magnitude_3(:), 'b-', 'LineWidth', 1.5);
  plot(t, error_magnitude_4(:), 'm-', 'LineWidth', 1.5);
  xlabel('时间 (s)'); ylabel('相对距离误差 (m)');
  title('跟随者与虚拟载具的相对距离误差');
  legend('跟随1', '跟随2', '跟随3', '跟随4');
  grid on;
end

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
    agent_theta_kp = unwrapToPi(agent_theta_kp); % 角度区间回正
end

% 多智能体辅助函数3：回正角度的输出，限定在区间内
function angle = unwrapToPi(angle)
    pi = 3.1415926;
    if angle > pi
        angle = angle - 2*pi;
    elseif angle < -pi
        angle = angle + 2*pi;
    end
end

% 多智能体辅助函数4：计算幽灵载具在k时刻的期望位置，以及虚拟线速度和虚拟角速度
% 需要说明的一点是，由于通信和逻辑关系的时间差，；领导者在k时刻计算出对应virtual在k时刻的期望位置
% 但是和虚拟载具在k-1时刻做差计算，得到的应该是k-1到k时刻的速度和角速度，也即输出是virtual.v(k-1)和virtual.w(k-1)
% 之后follower在k时刻根据virtual在k时刻的位置和virtual在k-1时刻的速度和角速度进行控制
% 原来的source.m在时间轴上其实是有一定的逻辑问题，这里加以更正
% here, km means k minus 1,kp means k plus 1
function [vir_x_k, vir_y_k, vir_theta_k, vir_v_km, vir_w_km] = ...
    compute_virtual_vehicle(hx_hat_k, hy_hat_k, leader_x_k, leader_y_k, leader_theta_k, vir_x_km, vir_y_km, vir_theta_km,dt)
    % 先根据lader的朝向角计算幽灵载具在世界坐标系下的位置
    hx_k = leader_x_k + cos(leader_theta_k) * hx_hat_k - sin(leader_theta_k) * hy_hat_k;
    hy_k = leader_y_k + sin(leader_theta_k) * hx_hat_k + cos(leader_theta_k) * hy_hat_k;
    vir_x_k = hx_k;vir_y_k = hy_k;
    % 计算幽灵载具的角朝向（基于前一时刻的位置）
    dx_rp = (vir_x_k - vir_x_km)/dt;
    dy_rp = (vir_y_k - vir_y_km)/dt;
    vir_theta_k = atan2(dy_rp, dx_rp);
    % 计算幽灵载具的线速度和角速度（基于前一时刻的位置） /* 这里之后的改进方向，使用曲率代替直接求导来计算角速度，对比两者给出的参考值 在同一算法条件下 的追踪效果 */
    vir_v_km = sqrt(dx_rp^2 + dy_rp^2);
    vir_w_km = (vir_theta_k - vir_theta_km);
end

% 多智能体辅助函数4_ver2：计算幽灵载具在k时刻的期望位置，以及虚拟线速度和虚拟角速度
% 使用曲率公式来计算角速度，线速度求解方法不变
function [vir_x_k, vir_y_k, vir_theta_k, vir_v_km, vir_w_km] = ...
    compute_virtual_vehicle_ver2(hx_hat_k, hy_hat_k, leader_x_k, leader_y_k, leader_theta_k, ...
                                vir_x_km, vir_y_km, vir_theta_km, ...
                                dt)
    % 先根据lader的朝向角计算幽灵载具在世界坐标系下的位置
    hx_k = leader_x_k + cos(leader_theta_k) * hx_hat_k - sin(leader_theta_k) * hy_hat_k;
    hy_k = leader_y_k + sin(leader_theta_k) * hx_hat_k + cos(leader_theta_k) * hy_hat_k;
    vir_x_k = hx_k;vir_y_k = hy_k;
    % 计算幽灵载具的角朝向（基于前一时刻的位置）
    dx_rp = (vir_x_k - vir_x_km)/dt;
    dy_rp = (vir_y_k - vir_y_km)/dt;
    vir_theta_k = atan2(dy_rp, dx_rp);
    % 计算幽灵载具的线速度和角速度（基于前一时刻的位置） /* 使用曲率公式来计算角速度 */
    vir_v_km = sqrt(dx_rp^2 + dy_rp^2);
    % 曲率公式：kappa = (x'y'' - y'x'') / ( (x'^2 + y'^2)^(3/2) )
    % 其中 x' 和 y' 是一阶导数，x'' 和 y'' 是二阶导数
    ddx_rp = (dx_rp - (vir_x_km - (vir_x_km - dx_rp*dt))/dt)/dt; % 二阶导数近似
    ddy_rp = (dy_rp - (vir_y_km - (vir_y_km - dy_rp*dt))/dt)/dt; % 二阶导数近似
    curvature = (dx_rp * ddy_rp - dy_rp * ddx_rp) / ((dx_rp^2 + dy_rp^2)^(3/2) + 1e-6); % 防止除零\turning_radius
    vir_w_km = curvature * vir_v_km;
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
    Dtheta_vp = unwrapToPi(Dtheta_vp);
end

% 多智能体辅助函数6：追踪效果测试算法
function [follower_v, follower_w] = ...
    tracking_test_control_law(Dx_vp, Dy_vp, Dtheta_vp, v_max, w_max)
    % 简单比例控制律
    kp_pos = 1; kp_angle = 1;
    vp = kp_pos * sqrt(Dx_vp^2 + Dy_vp^2);
    wp = kp_angle * Dtheta_vp;
    follower_v = max(0, min(v_max, vp)); % 速度约束
    follower_w = max(-w_max, min(w_max, wp)); % 角速度约束
end

%% TVF控制逻辑
% 辅助函数A：TVF控制律
% 时间轴问题：根据上一时刻传回的virtual_v和follower_theta计算当前时刻的控制输入
function [follower_v, follower_w] = ...
TVF_control_law(virtual_v, Dtheta_vp, ...
                k1, k2, k3, k4, ...
                Dx_vp, Dy_vp, v_max, w_max)
    % 控制律
    D_p = max(v_max, 1) + max(w_max, 1) * sqrt(Dx_vp^2 + Dy_vp^2);
    s_p = (k4 * Dy_vp / D_p) + Dtheta_vp;
    %vp = virtual_v;
    vp = virtual_v + k1 * Dx_vp / sqrt(Dx_vp^2 + Dy_vp^2 + k2);
    %wp = Dtheta_vp;
    wp = k3 * sign(s_p);
    follower_v = max(-v_max, min(v_max, vp)); % 速度约束
    follower_w = max(-w_max, min(w_max, wp)); % 角速度约束
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