%% 从 saves 中读取卡尔曼仿真数据并生成动画
% 默认读取最新的 kalman-test-*.mat；也可以在下方指定文件名。

clear; close all; clc;

%% 01 项目路径
script_full_path = mfilename('fullpath');
if isempty(script_full_path)
    project_root = pwd;
else
    project_root = fileparts(script_full_path);
end
auxiliary_folder = fullfile(project_root, 'auxiliary');
saves_folder = fullfile(project_root, 'saves');
addpath(auxiliary_folder);

if ~isfolder(saves_folder)
    error('没有找到仿真存档目录：%s', saves_folder);
end

%% 02 数据文件选择
% 留空表示读取 saves 中修改时间最新的 MAT；也可填入完整文件名或绝对路径。
selected_mat_filename = '';

available_mat_files = dir(fullfile(saves_folder, 'kalman-test-*.mat'));
if isempty(available_mat_files)
    error('saves 中没有 kalman-test-*.mat，请先运行仿真程序。');
end

if isempty(selected_mat_filename)
    [~, newest_file_index] = max([available_mat_files.datenum]);
    selected_mat_path = fullfile(available_mat_files(newest_file_index).folder, ...
        available_mat_files(newest_file_index).name);
else
    if isfile(selected_mat_filename)
        selected_mat_path = selected_mat_filename;
    else
        selected_mat_path = fullfile(saves_folder, selected_mat_filename);
    end
    if ~isfile(selected_mat_path)
        error('指定的 MAT 文件不存在：%s', selected_mat_path);
    end
end

fprintf('读取仿真数据：%s\n', selected_mat_path);
simulation_data = load(selected_mat_path);

%% 03 动画内容开关
show_agent_panel = 1;                 % 左侧实时智能体区域
show_kalman_panels = 1;               % 右侧三组卡尔曼概率分布

show_true_leader = 1;
show_noisy_leader_observation = 0;
show_posterior_leader = 0;
show_preview_leader = 1;
show_formation_reference_agents = 0;  % 默认不显示四个 virtual
show_followers = 1;
show_agent_orientation = 1;

probability_display_axis = 'x';       % 'x' 或 'y'
frame_stride = 10;
gif_delay_time = 0.1;

% 概率曲线视觉参数。
prior_distribution_color = [0.85, 0.15, 0.15];
observation_distribution_color = [0.15, 0.35, 0.90];
posterior_distribution_color = [0.10, 0.65, 0.25];
preview_distribution_color = [0.55, 0.00, 0.00];
active_distribution_opacity = 0.70;
context_distribution_opacity = 0.18;

%% 04 读取动画所需字段
required_fields = {'t', 'leader', 'leader_observation', 'leader_prior', ...
    'leader_posterior', 'leader_preview', 'follower_1', 'follower_2', ...
    'follower_3', 'follower_4', 'virtual_1', 'virtual_2', 'virtual_3', ...
    'virtual_4', 'preview_horizon', 'preview_used_for_control', ...
    'kalman_assumed_observation_std_x', 'kalman_assumed_observation_std_y'};
for field_index = 1:numel(required_fields)
    if ~isfield(simulation_data, required_fields{field_index})
        error('MAT 文件缺少动画所需字段：%s', required_fields{field_index});
    end
end

t = simulation_data.t;
leader = simulation_data.leader;
leader_observation = simulation_data.leader_observation;
leader_prior = simulation_data.leader_prior;
leader_posterior = simulation_data.leader_posterior;
leader_preview = simulation_data.leader_preview;
follower_1 = simulation_data.follower_1;
follower_2 = simulation_data.follower_2;
follower_3 = simulation_data.follower_3;
follower_4 = simulation_data.follower_4;
virtual_1 = simulation_data.virtual_1;
virtual_2 = simulation_data.virtual_2;
virtual_3 = simulation_data.virtual_3;
virtual_4 = simulation_data.virtual_4;
preview_horizon = simulation_data.preview_horizon;
preview_used_for_control = simulation_data.preview_used_for_control;
kalman_assumed_observation_std_x = simulation_data.kalman_assumed_observation_std_x;
kalman_assumed_observation_std_y = simulation_data.kalman_assumed_observation_std_y;

animation_timestamp = datestr(now, 'yy-mm-dd-HHMMSS');
gif_filename = fullfile(saves_folder, sprintf('kalman-test-%s.gif', animation_timestamp));
frame_indices = 1:frame_stride:numel(t);
if frame_indices(end) ~= numel(t)
    frame_indices(end+1) = numel(t);
end

%% 05 逐帧生成动画
fprintf('生成动画：%s\n', gif_filename);
animation_figure = figure('Position', [40, 40, 1600, 900], 'Color', 'w');

for frame_number = 1:numel(frame_indices)
    k = frame_indices(frame_number);
    clf(animation_figure);

    %% 左侧实时智能体
    trajectory_axes = subplot(3, 2, [1, 3, 5]);
    if show_agent_panel
        hold(trajectory_axes, 'on');
        legend_handles = gobjects(0);
        legend_labels = {};

        if show_true_leader
            true_leader_handle = plot(trajectory_axes, leader.x(1:k), leader.y(1:k), ...
                'k-', 'LineWidth', 2);
            legend_handles(end+1) = true_leader_handle;
            legend_labels{end+1} = '\rm真实领导者';
            scatter(trajectory_axes, leader.x(k), leader.y(k), 120, 'k', ...
                'filled', 'd', 'HandleVisibility', 'off');
        end

        if show_followers
            follower_1_handle = plot(trajectory_axes, follower_1.x(1:k), follower_1.y(1:k), ...
                'b-', 'LineWidth', 1.5);
            follower_2_handle = plot(trajectory_axes, follower_2.x(1:k), follower_2.y(1:k), ...
                'g-', 'LineWidth', 1.5);
            follower_3_handle = plot(trajectory_axes, follower_3.x(1:k), follower_3.y(1:k), ...
                'm-', 'LineWidth', 1.5);
            follower_4_handle = plot(trajectory_axes, follower_4.x(1:k), follower_4.y(1:k), ...
                'c-', 'LineWidth', 1.5);
            legend_handles = [legend_handles, follower_1_handle, follower_2_handle, ...
                follower_3_handle, follower_4_handle];
            legend_labels = [legend_labels, {'\rm跟随者1', '\rm跟随者2', ...
                '\rm跟随者3', '\rm跟随者4'}];
            scatter(trajectory_axes, follower_1.x(k), follower_1.y(k), 60, 'b', ...
                'filled', 'HandleVisibility', 'off');
            scatter(trajectory_axes, follower_2.x(k), follower_2.y(k), 60, 'g', ...
                'filled', 'HandleVisibility', 'off');
            scatter(trajectory_axes, follower_3.x(k), follower_3.y(k), 60, 'm', ...
                'filled', 'HandleVisibility', 'off');
            scatter(trajectory_axes, follower_4.x(k), follower_4.y(k), 60, 'c', ...
                'filled', 'HandleVisibility', 'off');
        end

        if show_formation_reference_agents
            virtual_handle = scatter(trajectory_axes, ...
                [virtual_1.x(k), virtual_2.x(k), virtual_3.x(k), virtual_4.x(k)], ...
                [virtual_1.y(k), virtual_2.y(k), virtual_3.y(k), virtual_4.y(k)], ...
                65, [0.75, 0.25, 0.10], 'filled', '^');
            legend_handles(end+1) = virtual_handle;
            legend_labels{end+1} = '\rm编队参考载具';
        end

        if show_noisy_leader_observation
            observation_handle = scatter(trajectory_axes, leader_observation.x(k), ...
                leader_observation.y(k), 55, observation_distribution_color, ...
                'x', 'LineWidth', 1.8);
            legend_handles(end+1) = observation_handle;
            legend_labels{end+1} = '\rm公共带噪声观测';
        end

        if show_posterior_leader
            posterior_handle = scatter(trajectory_axes, leader_posterior.state_x(1,k), ...
                leader_posterior.state_y(1,k), 90, posterior_distribution_color, ...
                'filled', 'o');
            legend_handles(end+1) = posterior_handle;
            legend_labels{end+1} = '\bf当前融合后验领导者（控制使用）';
        end

        if show_preview_leader
            preview_handle = scatter(trajectory_axes, leader_preview.x(k), ...
                leader_preview.y(k), 130, preview_distribution_color, 'filled', 'p');
            legend_handles(end+1) = preview_handle;
            if preview_used_for_control
                legend_labels{end+1} = '\bf算法前视输出（参与控制）';
            else
                legend_labels{end+1} = '\bf仅观测辅助：0.1 s预测领导者（不参与控制）';
            end
        end

        if show_agent_orientation && show_true_leader
            plot_robot_orientation_for_animation(trajectory_axes, leader.x(k), ...
                leader.y(k), leader.theta(k), 'k', 2);
        end

        xlabel(trajectory_axes, 'X坐标 (m)');
        ylabel(trajectory_axes, 'Y坐标 (m)');
        title(trajectory_axes, sprintf('编队运动轨迹  t = %.2f s', t(k)));
        if ~isempty(legend_handles)
            legend(trajectory_axes, legend_handles, legend_labels, ...
                'Location', 'best', 'Interpreter', 'tex', 'FontSize', 9);
        end
        grid(trajectory_axes, 'on');
        axis(trajectory_axes, 'equal');
    else
        axis(trajectory_axes, 'off');
        text(trajectory_axes, 0.5, 0.5, '左侧智能体显示已关闭', ...
            'HorizontalAlignment', 'center', 'Units', 'normalized');
    end

    %% 选择 X 或 Y 概率数据
    if strcmpi(probability_display_axis, 'x')
        displayed_axis_name = 'X';
        prior_mean = leader_prior.state_x(1,k);
        prior_variance = leader_prior.covariance_x(1,1,k);
        observation_mean = leader_observation.x(k);
        observation_variance = kalman_assumed_observation_std_x^2;
        posterior_mean = leader_posterior.state_x(1,k);
        posterior_variance = leader_posterior.covariance_x(1,1,k);
        preview_mean = leader_preview.x(k);
        preview_variance = leader_preview.variance_x(k);
            % 新存档优先读取标准化 vx；旧存档回退到兼容 state_x。
            if isfield(leader_posterior, 'vx')
                displayed_velocity = leader_posterior.vx(k);
            else
                displayed_velocity = leader_posterior.state_x(2,k);
            end
        displayed_kalman_gain = leader_posterior.kalman_gain_x(1,k);
    else
        displayed_axis_name = 'Y';
        prior_mean = leader_prior.state_y(1,k);
        prior_variance = leader_prior.covariance_y(1,1,k);
        observation_mean = leader_observation.y(k);
        observation_variance = kalman_assumed_observation_std_y^2;
        posterior_mean = leader_posterior.state_y(1,k);
        posterior_variance = leader_posterior.covariance_y(1,1,k);
        preview_mean = leader_preview.y(k);
        preview_variance = leader_preview.variance_y(k);
            % 新存档优先读取标准化 vy；旧存档回退到兼容 state_y。
            if isfield(leader_posterior, 'vy')
                displayed_velocity = leader_posterior.vy(k);
            else
                displayed_velocity = leader_posterior.state_y(2,k);
            end
        displayed_kalman_gain = leader_posterior.kalman_gain_y(1,k);
    end

    prior_variance = ensure_positive_variance_helper(prior_variance, 1e-12);
    observation_variance = ensure_positive_variance_helper(observation_variance, 1e-12);
    posterior_variance = ensure_positive_variance_helper(posterior_variance, 1e-12);
    preview_variance = ensure_positive_variance_helper(preview_variance, 1e-12);
    distribution_means = [prior_mean, observation_mean, posterior_mean, preview_mean];
    distribution_stds = sqrt([prior_variance, observation_variance, ...
        posterior_variance, preview_variance]);
    grid_minimum = min(distribution_means - 4 * distribution_stds);
    grid_maximum = max(distribution_means + 4 * distribution_stds);
    if grid_maximum <= grid_minimum
        grid_maximum = grid_minimum + 1;
    end
    position_probability_grid = linspace(grid_minimum, grid_maximum, 300);
    prior_density = evaluate_normal_probability_density( ...
        position_probability_grid, prior_mean, prior_variance);
    observation_density = evaluate_normal_probability_density( ...
        position_probability_grid, observation_mean, observation_variance);
    posterior_density = evaluate_normal_probability_density( ...
        position_probability_grid, posterior_mean, posterior_variance);
    preview_density = evaluate_normal_probability_density( ...
        position_probability_grid, preview_mean, preview_variance);

    %% 右侧三组卡尔曼概率图
    prior_observation_axes = subplot(3, 2, 2);
    posterior_axes = subplot(3, 2, 4);
    preview_axes = subplot(3, 2, 6);
    if show_kalman_panels
        % 红色先验与蓝色观测。
        plot_color_and_transparency_helper(prior_observation_axes, ...
            position_probability_grid, prior_density, prior_distribution_color, ...
            active_distribution_opacity, '先验预测分布');
        plot_color_and_transparency_helper(prior_observation_axes, ...
            position_probability_grid, observation_density, observation_distribution_color, ...
            active_distribution_opacity, '观测分布');
        title(prior_observation_axes, sprintf(['先验与观测：%s轴  ', ...
            'mu_prior=%.3f, mu_observation=%.3f'], ...
            displayed_axis_name, prior_mean, observation_mean));
        xlabel(prior_observation_axes, sprintf('%s位置 (m)', displayed_axis_name));
        ylabel(prior_observation_axes, '概率密度');
        legend(prior_observation_axes, 'show', 'Location', 'best');
        grid(prior_observation_axes, 'on');

        % 淡化先验和观测，突出绿色融合后验。
        plot_color_and_transparency_helper(posterior_axes, ...
            position_probability_grid, prior_density, prior_distribution_color, ...
            context_distribution_opacity, '先验（背景）');
        plot_color_and_transparency_helper(posterior_axes, ...
            position_probability_grid, observation_density, observation_distribution_color, ...
            context_distribution_opacity, '观测（背景）');
        plot_color_and_transparency_helper(posterior_axes, ...
            position_probability_grid, posterior_density, posterior_distribution_color, ...
            active_distribution_opacity, '滤波后验分布');
        title(posterior_axes, sprintf('概率融合：K=%.3f, mu=%.3f, sigma=%.3f', ...
            displayed_kalman_gain, posterior_mean, sqrt(posterior_variance)));
        xlabel(posterior_axes, sprintf('%s位置 (m)', displayed_axis_name));
        ylabel(posterior_axes, '概率密度');
        legend(posterior_axes, 'show', 'Location', 'best');
        grid(posterior_axes, 'on');

        % 后验向前传播 preview_horizon，仅用于观察时不会进入控制。
        plot_color_and_transparency_helper(preview_axes, ...
            position_probability_grid, posterior_density, posterior_distribution_color, ...
            context_distribution_opacity, '当前后验（背景）');
        plot_color_and_transparency_helper(preview_axes, ...
            position_probability_grid, preview_density, preview_distribution_color, ...
            active_distribution_opacity, '前视分布');
        title(preview_axes, sprintf(['前视 %.2f s：v=%.3f m/s, ', ...
            'mu=%.3f, sigma=%.3f'], preview_horizon, displayed_velocity, ...
            preview_mean, sqrt(preview_variance)));
        xlabel(preview_axes, sprintf('%s位置 (m)', displayed_axis_name));
        ylabel(preview_axes, '概率密度');
        legend(preview_axes, 'show', 'Location', 'best');
        grid(preview_axes, 'on');
    else
        axis(prior_observation_axes, 'off');
        axis(posterior_axes, 'off');
        axis(preview_axes, 'off');
        text(posterior_axes, 0.5, 0.5, '右侧卡尔曼概率显示已关闭', ...
            'HorizontalAlignment', 'center', 'Units', 'normalized');
    end

    drawnow;
    frame = getframe(animation_figure);
    image_data = frame2im(frame);
    [indexed_image, color_map] = rgb2ind(image_data, 256);
    if frame_number == 1
        imwrite(indexed_image, color_map, gif_filename, 'gif', ...
            'Loopcount', inf, 'DelayTime', gif_delay_time);
    else
        imwrite(indexed_image, color_map, gif_filename, 'gif', ...
            'WriteMode', 'append', 'DelayTime', gif_delay_time);
    end
end

fprintf('动画已保存为 %s\n', gif_filename);

function plot_robot_orientation_for_animation(axes_handle, x, y, theta, color, linewidth)
%PLOT_ROBOT_ORIENTATION_FOR_ANIMATION 绘制智能体当前朝向，不加入图例。
arrow_length = 1.5;
plot(axes_handle, [x, x + arrow_length * cos(theta)], ...
    [y, y + arrow_length * sin(theta)], color, ...
    'LineWidth', linewidth, 'HandleVisibility', 'off');
end
