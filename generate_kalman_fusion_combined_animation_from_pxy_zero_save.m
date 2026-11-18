%% 从 Pxy=0 存档生成“融合痕迹”组合动画
% 本程序只读取既有 MAT 数据，不重新运行仿真，也不改变 Kalman 参数。
%
% 画面布局：
%   左侧：领导者与跟随者的实时轨迹；
%   右上：XY 位置二维高斯的三维叠加图；
%         紫色后验先绘制且不透明，红色先验和蓝色观测后绘制且半透明；
%   右中：v 速度先验、后验位置窗口推导速度、现有速度后验和真实值；
%   右下：后验位置与真实位置的误差模值 d 随时间变化。
%
% 默认 render_mode='animation'：直接生成完整 GIF。
% 正式位置图使用 8 sigma 范围，并将三个均值之间的显示距离放大 8 倍；
% 该放大只用于观察融合位置，不修改 MAT 数据、均值或协方差。
% 如需重新调试静态画面，可将 render_mode 改为 'preview_scales'。

clearvars -except render_mode selected_mat_filename selected_preview_time ...
    chosen_probability_sigma_span preview_probability_sigma_spans ...
    position_mean_separation_magnification;
close all; clc;

%% 01 可调运行方式
if ~exist('render_mode', 'var') || isempty(render_mode)
    render_mode = 'animation';  % 'animation' 或 'preview_scales'
end
if ~exist('selected_mat_filename', 'var')
    selected_mat_filename = '';     % 留空：读取最新 kalman-test-pxy-zero-*.mat
end
if ~exist('selected_preview_time', 'var') || isempty(selected_preview_time)
    selected_preview_time = 10.0;   % 三张尺度预览所用的仿真时刻 (s)
end
if ~exist('chosen_probability_sigma_span', 'var') || ...
        isempty(chosen_probability_sigma_span)
    chosen_probability_sigma_span = 8.0; % 正式动画采用 8 sigma
end
if ~exist('preview_probability_sigma_spans', 'var') || ...
        isempty(preview_probability_sigma_spans)
    preview_probability_sigma_spans = 8.0;
end
if ~exist('position_mean_separation_magnification', 'var') || ...
        isempty(position_mean_separation_magnification)
    position_mean_separation_magnification = 8.0;
end
probability_grid_point_count = 71;
frame_stride = 10;                 % dt=0.01 s 时，每 0.1 s 一帧
gif_delay_time = 0.1;
velocity_observation_window_length = 11; % 最近 11 个后验位置点，约 0.1 s

%% 02 项目路径与数据文件
script_full_path = mfilename('fullpath');
if isempty(script_full_path)
    project_root = pwd;
else
    project_root = fileparts(script_full_path);
end
auxiliary_folder = fullfile(project_root, 'auxiliary');
saves_folder = fullfile(project_root, 'saves');
results_folder = fullfile(project_root, 'results');
addpath(auxiliary_folder);

if ~isfolder(saves_folder)
    error('没有找到存档目录：%s', saves_folder);
end
if ~isfolder(results_folder)
    mkdir(results_folder);
end

available_mat_files = dir(fullfile(saves_folder, 'kalman-test-pxy-zero-*.mat'));
if isempty(available_mat_files)
    error('saves 中没有 kalman-test-pxy-zero-*.mat。');
end

if isempty(selected_mat_filename)
    [~, newest_file_index] = max([available_mat_files.datenum]);
    selected_mat_path = fullfile(available_mat_files(newest_file_index).folder, ...
        available_mat_files(newest_file_index).name);
elseif isfile(selected_mat_filename)
    selected_mat_path = selected_mat_filename;
else
    selected_mat_path = fullfile(saves_folder, selected_mat_filename);
end
if ~isfile(selected_mat_path)
    error('指定的 MAT 文件不存在：%s', selected_mat_path);
end

simulation_data = load(selected_mat_path);
fprintf('读取存档：%s\n', selected_mat_path);

%% 03 数据完整性检查
required_fields = {'t', 'dt', 'leader', 'leader_observation', 'leader_prior', ...
    'leader_posterior', 'leader_preview', 'follower_1', 'follower_2', ...
    'follower_3', 'follower_4', 'preview_horizon', ...
    'preview_used_for_control', 'kalman_assumed_observation_std_x', ...
    'kalman_assumed_observation_std_y'};
for field_index = 1:numel(required_fields)
    if ~isfield(simulation_data, required_fields{field_index})
        error('MAT 文件缺少渲染所需字段：%s', required_fields{field_index});
    end
end
if ~isfield(simulation_data.leader_prior, 'state') || ...
        ~isfield(simulation_data.leader_prior, 'covariance') || ...
        ~isfield(simulation_data.leader_posterior, 'state') || ...
        ~isfield(simulation_data.leader_posterior, 'covariance')
    error('存档不是统一 [x;y;v] Kalman 数据结构。');
end

prior_pxy = reshape(simulation_data.leader_prior.covariance(1,2,:), [], 1);
posterior_pxy = reshape(simulation_data.leader_posterior.covariance(1,2,:), [], 1);
maximum_absolute_pxy = max(abs([prior_pxy; posterior_pxy]));
if maximum_absolute_pxy > 1e-12
    error('所选存档不是严格 Pxy=0 数据；最大 |Pxy|=%.6g。', ...
        maximum_absolute_pxy);
end
fprintf('最大 |Pxy|：%.3e\n', maximum_absolute_pxy);

%% 04 显示内容开关
display_options.show_agent_panel = 1;
display_options.show_probability_panels = 1;
display_options.show_true_leader = 1;
display_options.show_noisy_leader_observation = 0;
display_options.show_posterior_leader = 0;
display_options.show_preview_leader = 1;
display_options.show_followers = 1;
display_options.show_agent_orientation = 1;

%% 05 配色、透明度和视角
plot_style.prior_position_color = [0.86, 0.12, 0.12];      % 红色
plot_style.observation_position_color = [0.08, 0.32, 0.88];% 蓝色
plot_style.posterior_position_color = [0.52, 0.18, 0.68];  % 紫色
plot_style.prior_position_opacity = 0.32;
plot_style.observation_position_opacity = 0.32;
plot_style.posterior_position_opacity = 1.00;
plot_style.view_azimuth_deg = 45;
plot_style.view_elevation_deg = 30;

% 速度图不使用红、蓝、紫。后验颜色由先验和诊断速度两色平均得到。
plot_style.prior_velocity_color = [0.93, 0.55, 0.08];      % 琥珀色
plot_style.observation_velocity_color = [0.03, 0.62, 0.65];% 青绿色
plot_style.posterior_velocity_color = 0.5 * (...
    plot_style.prior_velocity_color + plot_style.observation_velocity_color);
plot_style.velocity_context_opacity = 0.28;
plot_style.velocity_posterior_opacity = 0.72;
plot_style.true_velocity_color = [0, 0, 0];

plot_style.probability_grid_point_count = probability_grid_point_count;
plot_style.position_mean_separation_magnification = ...
    position_mean_separation_magnification;
plot_style.position_error_color = [0, 0, 0];

%% 06 预先计算固定坐标范围和误差数据
time = simulation_data.t(:).';
time_sample_count = numel(time);
posterior_position_error = hypot(...
    simulation_data.leader_posterior.state(1,1:time_sample_count) ...
    - simulation_data.leader.x(1:time_sample_count), ...
    simulation_data.leader_posterior.state(2,1:time_sample_count) ...
    - simulation_data.leader.y(1:time_sample_count));
plot_style.maximum_position_error = max(posterior_position_error);
if plot_style.maximum_position_error <= 0
    plot_style.maximum_position_error = 1;
end

% 只用于动画诊断：从最近若干后验位置点和同构圆弧运动学反求速度。
% 该速度观测没有参与当前 MAT 文件中已经完成的 Kalman 后验更新。
posterior_position_velocity_observation = ...
    derive_velocity_observation_from_posterior_positions(...
    simulation_data, velocity_observation_window_length);

all_trajectory_x = [simulation_data.leader.x(1:time_sample_count), ...
    simulation_data.follower_1.x(1:time_sample_count), ...
    simulation_data.follower_2.x(1:time_sample_count), ...
    simulation_data.follower_3.x(1:time_sample_count), ...
    simulation_data.follower_4.x(1:time_sample_count)];
all_trajectory_y = [simulation_data.leader.y(1:time_sample_count), ...
    simulation_data.follower_1.y(1:time_sample_count), ...
    simulation_data.follower_2.y(1:time_sample_count), ...
    simulation_data.follower_3.y(1:time_sample_count), ...
    simulation_data.follower_4.y(1:time_sample_count)];
trajectory_span = max([max(all_trajectory_x) - min(all_trajectory_x), ...
    max(all_trajectory_y) - min(all_trajectory_y), 1]);
trajectory_margin = 0.06 * trajectory_span;
plot_style.trajectory_x_limits = [min(all_trajectory_x) - trajectory_margin, ...
    max(all_trajectory_x) + trajectory_margin];
plot_style.trajectory_y_limits = [min(all_trajectory_y) - trajectory_margin, ...
    max(all_trajectory_y) + trajectory_margin];

observation_covariance_xy = diag([...
    simulation_data.kalman_assumed_observation_std_x^2, ...
    simulation_data.kalman_assumed_observation_std_y^2]);

%% 07 生成尺度预览或完整动画
render_timestamp = datestr(now, 'yy-mm-dd-HHMMSS');

if strcmpi(render_mode, 'preview_scales')
    [~, preview_index] = min(abs(time - selected_preview_time));
    fprintf('尺度预览时刻：%.3f s（索引 %d）\n', time(preview_index), preview_index);

    for scale_index = 1:numel(preview_probability_sigma_spans)
        sigma_span = preview_probability_sigma_spans(scale_index);
        preview_figure = figure('Position', [40, 40, 1600, 900], ...
            'Color', 'w', 'Renderer', 'opengl');
        render_complete_frame(preview_figure, preview_index, sigma_span, ...
            simulation_data, observation_covariance_xy, posterior_position_error, ...
            posterior_position_velocity_observation, ...
            display_options, plot_style);
        drawnow;

        sigma_label = strrep(sprintf('%.1f', sigma_span), '.', 'p');
        preview_filename = fullfile(results_folder, sprintf(...
            'kalman-pxy-zero-fusion-preview-%ssigma-%s.png', ...
            sigma_label, render_timestamp));
        print(preview_figure, preview_filename, '-dpng', '-r150');
        close(preview_figure);
        fprintf('已写入尺度预览：%s\n', preview_filename);
    end

    fprintf(['预览完成。确认尺度后，将 render_mode 改为 animation，', ...
        '并设置 chosen_probability_sigma_span。\n']);

elseif strcmpi(render_mode, 'animation')
    animation_filename = fullfile(saves_folder, sprintf(...
        'kalman-test-pxy-zero-fusion-combined-%s.gif', render_timestamp));
    frame_indices = 1:frame_stride:numel(time);
    if frame_indices(end) ~= numel(time)
        frame_indices(end+1) = numel(time);
    end

    fprintf('输出动画：%s\n', animation_filename);
    fprintf('位置概率范围：%.1f sigma\n', chosen_probability_sigma_span);
    fprintf('位置均值间距视觉放大：%.1f 倍\n', ...
        position_mean_separation_magnification);
    fprintf('待渲染帧数：%d\n', numel(frame_indices));

    animation_figure = figure('Position', [40, 40, 1600, 900], ...
        'Color', 'w', 'Renderer', 'opengl');
    for frame_number = 1:numel(frame_indices)
        k = frame_indices(frame_number);
        clf(animation_figure);
        render_complete_frame(animation_figure, k, ...
            chosen_probability_sigma_span, simulation_data, ...
            observation_covariance_xy, posterior_position_error, ...
            posterior_position_velocity_observation, ...
            display_options, plot_style);
        drawnow;

        frame = getframe(animation_figure);
        image_data = frame2im(frame);
        [indexed_image, color_map] = rgb2ind(image_data, 256);
        if frame_number == 1
            imwrite(indexed_image, color_map, animation_filename, 'gif', ...
                'Loopcount', inf, 'DelayTime', gif_delay_time);
        else
            imwrite(indexed_image, color_map, animation_filename, 'gif', ...
                'WriteMode', 'append', 'DelayTime', gif_delay_time);
        end

        if frame_number == 1 || mod(frame_number, 20) == 0 || ...
                frame_number == numel(frame_indices)
            fprintf('进度：%d/%d 帧，仿真时刻 %.2f s\n', ...
                frame_number, numel(frame_indices), time(k));
        end
    end
    close(animation_figure);
    fprintf('动画已完成：%s\n', animation_filename);
else
    error('未知 render_mode：%s。只能使用 preview_scales 或 animation。', ...
        render_mode);
end

%% 局部渲染函数
function render_complete_frame(figure_handle, k, probability_sigma_span, ...
        data, observation_covariance_xy, posterior_position_error, ...
        posterior_position_velocity_observation, ...
        display_options, plot_style)
%RENDER_COMPLETE_FRAME 绘制一帧完整的 3x2 布局。

time = data.t(:).';

%% 左侧：实时智能体轨迹
trajectory_axes = subplot(3, 2, [1, 3, 5], 'Parent', figure_handle);
if display_options.show_agent_panel
    hold(trajectory_axes, 'on');
    legend_handles = gobjects(0);
    legend_labels = {};

    if display_options.show_true_leader
        leader_handle = plot(trajectory_axes, data.leader.x(1:k), ...
            data.leader.y(1:k), 'k-', 'LineWidth', 2);
        legend_handles(end+1) = leader_handle;
        legend_labels{end+1} = '真实领导者';
        scatter(trajectory_axes, data.leader.x(k), data.leader.y(k), ...
            110, 'k', 'filled', 'd', 'HandleVisibility', 'off');
    end

    if display_options.show_followers
        follower_colors = {'b', 'g', 'm', 'c'};
        follower_data = {data.follower_1, data.follower_2, ...
            data.follower_3, data.follower_4};
        for follower_index = 1:4
            follower_handle = plot(trajectory_axes, ...
                follower_data{follower_index}.x(1:k), ...
                follower_data{follower_index}.y(1:k), ...
                [follower_colors{follower_index}, '-'], 'LineWidth', 1.5);
            legend_handles(end+1) = follower_handle;
            legend_labels{end+1} = sprintf('跟随者%d', follower_index);
            scatter(trajectory_axes, follower_data{follower_index}.x(k), ...
                follower_data{follower_index}.y(k), 55, ...
                follower_colors{follower_index}, 'filled', ...
                'HandleVisibility', 'off');
        end
    end

    if display_options.show_noisy_leader_observation
        observation_handle = scatter(trajectory_axes, ...
            data.leader_observation.x(k), data.leader_observation.y(k), ...
            55, plot_style.observation_position_color, 'x', 'LineWidth', 1.8);
        legend_handles(end+1) = observation_handle;
        legend_labels{end+1} = '公共带噪声观测';
    end

    if display_options.show_posterior_leader
        posterior_handle = scatter(trajectory_axes, ...
            data.leader_posterior.state(1,k), ...
            data.leader_posterior.state(2,k), 90, ...
            plot_style.posterior_position_color, 'filled', 'o');
        legend_handles(end+1) = posterior_handle;
        legend_labels{end+1} = '\bf算法本体：当前后验领导者（控制使用）';
    end

    if display_options.show_preview_leader
        preview_handle = scatter(trajectory_axes, data.leader_preview.x(k), ...
            data.leader_preview.y(k), 130, [0.55, 0.00, 0.00], ...
            'filled', 'p');
        legend_handles(end+1) = preview_handle;
        if data.preview_used_for_control
            legend_labels{end+1} = '\bf算法前视输出（参与控制）';
        else
            legend_labels{end+1} = ['\bf仅观测辅助：', ...
                num2str(data.preview_horizon, '%.1f'), ...
                ' s预测领导者（不参与控制）'];
        end
    end

    if display_options.show_agent_orientation && ...
            display_options.show_true_leader
        plot_robot_orientation(trajectory_axes, data.leader.x(k), ...
            data.leader.y(k), data.leader.theta(k), 'k', 2);
    end

    xlim(trajectory_axes, plot_style.trajectory_x_limits);
    ylim(trajectory_axes, plot_style.trajectory_y_limits);
    axis(trajectory_axes, 'equal');
    grid(trajectory_axes, 'on');
    xlabel(trajectory_axes, 'X坐标 (m)');
    ylabel(trajectory_axes, 'Y坐标 (m)');
    title(trajectory_axes, sprintf('编队运动轨迹  t = %.2f s', time(k)));
    if ~isempty(legend_handles)
        legend(trajectory_axes, legend_handles, legend_labels, ...
            'Location', 'best', 'Interpreter', 'tex', 'FontSize', 8);
    end
else
    axis(trajectory_axes, 'off');
    text(trajectory_axes, 0.5, 0.5, '左侧智能体显示已关闭', ...
        'HorizontalAlignment', 'center', 'Units', 'normalized');
end

%% 右上：三个 XY 二维高斯分布叠加
position_probability_axes = subplot(3, 2, 2, 'Parent', figure_handle);
if display_options.show_probability_panels
    prior_mean_xy = data.leader_prior.state(1:2,k);
    prior_covariance_xy = data.leader_prior.covariance(1:2,1:2,k);
    observation_mean_xy = [data.leader_observation.x(k); ...
        data.leader_observation.y(k)];
    posterior_mean_xy = data.leader_posterior.state(1:2,k);
    posterior_covariance_xy = data.leader_posterior.covariance(1:2,1:2,k);

    % 只放大三个均值之间的距离，协方差和曲面宽度保持原值。
    % 后验均值作为显示参考点；倍率为 1 时完全保持真实坐标。
    mean_distance_scale = plot_style.position_mean_separation_magnification;
    displayed_posterior_mean_xy = posterior_mean_xy;
    displayed_prior_mean_xy = posterior_mean_xy + mean_distance_scale * (...
        prior_mean_xy - posterior_mean_xy);
    displayed_observation_mean_xy = posterior_mean_xy + mean_distance_scale * (...
        observation_mean_xy - posterior_mean_xy);

    [x_grid, y_grid] = build_common_xy_grid(displayed_prior_mean_xy, ...
        prior_covariance_xy, displayed_observation_mean_xy, ...
        observation_covariance_xy, displayed_posterior_mean_xy, ...
        posterior_covariance_xy, probability_sigma_span, ...
        plot_style.probability_grid_point_count);
    [x_mesh, y_mesh] = meshgrid(x_grid, y_grid);
    prior_density = evaluate_2d_gaussian_density(x_mesh, y_mesh, ...
        displayed_prior_mean_xy, prior_covariance_xy);
    observation_density = evaluate_2d_gaussian_density(x_mesh, y_mesh, ...
        displayed_observation_mean_xy, observation_covariance_xy);
    posterior_density = evaluate_2d_gaussian_density(x_mesh, y_mesh, ...
        displayed_posterior_mean_xy, posterior_covariance_xy);

    hold(position_probability_axes, 'on');
    % 按用户要求：紫色后验最先渲染，随后再叠加半透明先验和观测。
    surf(position_probability_axes, x_mesh, y_mesh, posterior_density, ...
        'FaceColor', plot_style.posterior_position_color, ...
        'FaceAlpha', plot_style.posterior_position_opacity, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    surf(position_probability_axes, x_mesh, y_mesh, prior_density, ...
        'FaceColor', plot_style.prior_position_color, ...
        'FaceAlpha', plot_style.prior_position_opacity, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    surf(position_probability_axes, x_mesh, y_mesh, observation_density, ...
        'FaceColor', plot_style.observation_position_color, ...
        'FaceAlpha', plot_style.observation_position_opacity, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');

    contour3(position_probability_axes, x_mesh, y_mesh, posterior_density, ...
        7, 'LineColor', 0.72 * plot_style.posterior_position_color, ...
        'LineWidth', 0.55, 'HandleVisibility', 'off');
    contour3(position_probability_axes, x_mesh, y_mesh, prior_density, ...
        7, 'LineColor', plot_style.prior_position_color, ...
        'LineWidth', 0.45, 'HandleVisibility', 'off');
    contour3(position_probability_axes, x_mesh, y_mesh, observation_density, ...
        7, 'LineColor', plot_style.observation_position_color, ...
        'LineWidth', 0.45, 'HandleVisibility', 'off');

    % 使用虚拟线段建立稳定图例，避免透明曲面在图例中显示不一致。
    prior_legend = plot3(position_probability_axes, nan, nan, nan, '-', ...
        'Color', plot_style.prior_position_color, 'LineWidth', 3);
    observation_legend = plot3(position_probability_axes, nan, nan, nan, '-', ...
        'Color', plot_style.observation_position_color, 'LineWidth', 3);
    posterior_legend = plot3(position_probability_axes, nan, nan, nan, '-', ...
        'Color', plot_style.posterior_position_color, 'LineWidth', 3);
    legend(position_probability_axes, ...
        [prior_legend, observation_legend, posterior_legend], ...
        {'先验（半透明）', '观测（半透明）', '融合后验'}, ...
        'Location', 'northeast', 'FontSize', 7.5);

    xlim(position_probability_axes, [x_grid(1), x_grid(end)]);
    ylim(position_probability_axes, [y_grid(1), y_grid(end)]);
    maximum_density = max([prior_density(:); observation_density(:); ...
        posterior_density(:)]);
    zlim(position_probability_axes, [0, 1.06 * maximum_density]);
    view(position_probability_axes, plot_style.view_azimuth_deg, ...
        plot_style.view_elevation_deg);
    set(position_probability_axes, 'Projection', 'perspective', ...
        'FontSize', 7.5, 'GridAlpha', 0.18, 'LineWidth', 0.7);
    grid(position_probability_axes, 'on');
    box(position_probability_axes, 'on');
    if abs(mean_distance_scale - 1) > 1e-12
        xlabel(position_probability_axes, sprintf(...
            '显示X位置 (m，均值间距\\times%.1f)', mean_distance_scale));
        ylabel(position_probability_axes, sprintf(...
            '显示Y位置 (m，均值间距\\times%.1f)', mean_distance_scale));
    else
        xlabel(position_probability_axes, 'X位置 (m)');
        ylabel(position_probability_axes, 'Y位置 (m)');
    end
    zlabel(position_probability_axes, '概率密度');
    title(position_probability_axes, sprintf(...
        ['XY位置高斯融合（%.1f\\sigma，', ...
        '均值间距视觉放大\\times%.1f）'], ...
        probability_sigma_span, mean_distance_scale));

    %% 右中：速度先验、后验位置推导速度和现有速度后验
    velocity_axes = subplot(3, 2, 4, 'Parent', figure_handle);
    prior_velocity_mean = data.leader_prior.state(3,k);
    prior_velocity_variance = max(...
        data.leader_prior.covariance(3,3,k), 1e-12);
    posterior_velocity_mean = data.leader_posterior.state(3,k);
    posterior_velocity_variance = max(...
        data.leader_posterior.covariance(3,3,k), 1e-12);

    diagnostic_velocity_is_available = ...
        posterior_position_velocity_observation.is_available(k);
    diagnostic_velocity_mean = ...
        posterior_position_velocity_observation.mean(k);
    diagnostic_velocity_variance = ...
        posterior_position_velocity_observation.variance(k);
    true_velocity = data.leader.v(k);

    velocity_means = [prior_velocity_mean, posterior_velocity_mean];
    velocity_standard_deviations = sqrt([prior_velocity_variance, ...
        posterior_velocity_variance]);
    if diagnostic_velocity_is_available
        velocity_means(end+1) = diagnostic_velocity_mean;
        velocity_standard_deviations(end+1) = ...
            sqrt(diagnostic_velocity_variance);
    end
    velocity_grid_minimum = min(velocity_means - 4 * velocity_standard_deviations);
    velocity_grid_maximum = max(velocity_means + 4 * velocity_standard_deviations);
    velocity_reference_margin = max(0.08 * (...
        velocity_grid_maximum - velocity_grid_minimum), 0.05);
    velocity_grid_minimum = min(velocity_grid_minimum, ...
        true_velocity - velocity_reference_margin);
    velocity_grid_maximum = max(velocity_grid_maximum, ...
        true_velocity + velocity_reference_margin);
    if velocity_grid_maximum <= velocity_grid_minimum
        velocity_grid_maximum = velocity_grid_minimum + 1;
    end
    velocity_grid = linspace(velocity_grid_minimum, ...
        velocity_grid_maximum, 400);
    prior_velocity_density = evaluate_normal_probability_density(...
        velocity_grid, prior_velocity_mean, prior_velocity_variance);
    posterior_velocity_density = evaluate_normal_probability_density(...
        velocity_grid, posterior_velocity_mean, posterior_velocity_variance);

    % 先验与后验位置推导速度是背景，现有后验使用两色平均色突出显示。
    plot_color_and_transparency_helper(velocity_axes, velocity_grid, ...
        prior_velocity_density, plot_style.prior_velocity_color, ...
        plot_style.velocity_context_opacity, '速度先验');
    if diagnostic_velocity_is_available
        diagnostic_velocity_density = evaluate_normal_probability_density(...
            velocity_grid, diagnostic_velocity_mean, ...
            diagnostic_velocity_variance);
        plot_color_and_transparency_helper(velocity_axes, velocity_grid, ...
            diagnostic_velocity_density, ...
            plot_style.observation_velocity_color, ...
            plot_style.velocity_context_opacity, ...
            '后验位置窗口推导速度');
    end
    plot_color_and_transparency_helper(velocity_axes, velocity_grid, ...
        posterior_velocity_density, plot_style.posterior_velocity_color, ...
        plot_style.velocity_posterior_opacity, '现有 Kalman 速度后验');
    all_velocity_densities = [prior_velocity_density, ...
        posterior_velocity_density];
    if diagnostic_velocity_is_available
        all_velocity_densities = [all_velocity_densities, ...
            diagnostic_velocity_density];
    end
    maximum_velocity_density = max(all_velocity_densities);
    plot(velocity_axes, [true_velocity, true_velocity], ...
        [0, 1.03 * maximum_velocity_density], '--', ...
        'Color', plot_style.true_velocity_color, 'LineWidth', 1.4, ...
        'DisplayName', '真实领导者速度（评价基准）');
    grid(velocity_axes, 'on');
    xlabel(velocity_axes, '纵向速度 v (m/s)');
    ylabel(velocity_axes, '概率密度');
    if diagnostic_velocity_is_available
        title(velocity_axes, sprintf(...
            '速度状态对照：后验位置窗口推导值 %.3f m/s', ...
            diagnostic_velocity_mean));
    else
        title(velocity_axes, ...
            '速度状态对照：后验位置窗口样本尚不足');
    end
    legend(velocity_axes, 'show', 'Location', 'best', 'FontSize', 7.5);

    %% 右下：后验位置真实误差模值
    error_axes = subplot(3, 2, 6, 'Parent', figure_handle);
    plot(error_axes, time(1:k), posterior_position_error(1:k), ...
        'Color', plot_style.position_error_color, 'LineWidth', 1.7);
    hold(error_axes, 'on');
    plot(error_axes, time(k), posterior_position_error(k), 'ko', ...
        'MarkerFaceColor', 'k', 'MarkerSize', 4);
    xlim(error_axes, [time(1), time(end)]);
    ylim(error_axes, [0, 1.08 * plot_style.maximum_position_error]);
    grid(error_axes, 'on');
    xlabel(error_axes, '时间 t (s)');
    ylabel(error_axes, '误差模值 d (m)');
    title(error_axes, sprintf(...
        '后验--真实位置误差  d=%.5f m', posterior_position_error(k)));
else
    velocity_axes = subplot(3, 2, 4, 'Parent', figure_handle);
    error_axes = subplot(3, 2, 6, 'Parent', figure_handle);
    axis(position_probability_axes, 'off');
    axis(velocity_axes, 'off');
    axis(error_axes, 'off');
    text(velocity_axes, 0.5, 0.5, '右侧概率与误差显示已关闭', ...
        'HorizontalAlignment', 'center', 'Units', 'normalized');
end
end

function [x_grid, y_grid] = build_common_xy_grid(prior_mean, ...
        prior_covariance, observation_mean, observation_covariance, ...
        posterior_mean, posterior_covariance, sigma_span, grid_point_count)
%BUILD_COMMON_XY_GRID 按各分布边缘标准差建立统一 XY 显示范围。
means = [prior_mean(:), observation_mean(:), posterior_mean(:)];
standard_deviations = [sqrt(diag(prior_covariance)), ...
    sqrt(diag(observation_covariance)), sqrt(diag(posterior_covariance))];
x_minimum = min(means(1,:) - sigma_span * standard_deviations(1,:));
x_maximum = max(means(1,:) + sigma_span * standard_deviations(1,:));
y_minimum = min(means(2,:) - sigma_span * standard_deviations(2,:));
y_maximum = max(means(2,:) + sigma_span * standard_deviations(2,:));
if x_maximum <= x_minimum
    x_maximum = x_minimum + 1;
end
if y_maximum <= y_minimum
    y_maximum = y_minimum + 1;
end
x_grid = linspace(x_minimum, x_maximum, grid_point_count);
y_grid = linspace(y_minimum, y_maximum, grid_point_count);
end

function probability_density = evaluate_2d_gaussian_density(...
        x_mesh, y_mesh, mean_xy, covariance_xy)
%EVALUATE_2D_GAUSSIAN_DENSITY 手算二维高斯密度，不调用统计工具箱。
covariance_xy = 0.5 * (covariance_xy + covariance_xy.');
covariance_xx = max(covariance_xy(1,1), 1e-12);
covariance_yy = max(covariance_xy(2,2), 1e-12);
covariance_cross = covariance_xy(1,2);
covariance_determinant = covariance_xx * covariance_yy - covariance_cross^2;
if covariance_determinant <= 0
    error('二维位置协方差不是正定矩阵。');
end

position_error_x = x_mesh - mean_xy(1);
position_error_y = y_mesh - mean_xy(2);
mahalanobis_squared = (...
    covariance_yy * position_error_x.^2 ...
    - 2 * covariance_cross * position_error_x .* position_error_y ...
    + covariance_xx * position_error_y.^2) / covariance_determinant;
probability_density = exp(-0.5 * mahalanobis_squared) / ...
    (2 * pi * sqrt(covariance_determinant));
end

function velocity_observation = ...
        derive_velocity_observation_from_posterior_positions(data, window_length)
%DERIVE_VELOCITY_OBSERVATION_FROM_POSTERIOR_POSITIONS
% 使用最近 window_length 个后验位置点，通过同构圆弧运动学执行因果加权拟合。
% 该结果是动画诊断量，不是当前 MAT 中 Kalman 更新使用过的独立测量。

sample_count = numel(data.t);
velocity_observation.mean = nan(1, sample_count);
velocity_observation.variance = nan(1, sample_count);
velocity_observation.is_available = false(1, sample_count);
minimum_interval_count = 3;

for k = 2:sample_count
    first_position_index = max(1, k - window_length + 1);
    first_interval_index = first_position_index + 1;
    if k - first_interval_index + 1 < minimum_interval_count
        continue;
    end

    information_sum = 0;
    weighted_displacement_sum = 0;

    for j = first_interval_index:k
        theta_previous = data.leader.theta(j-1);
        angular_velocity_previous = data.leader.w(j-1);
        if abs(angular_velocity_previous) > 1e-8
            displacement_coefficient_x = (...
                sin(theta_previous + angular_velocity_previous * data.dt) ...
                - sin(theta_previous)) / angular_velocity_previous;
            displacement_coefficient_y = (...
                cos(theta_previous) ...
                - cos(theta_previous + angular_velocity_previous * data.dt)) ...
                / angular_velocity_previous;
        else
            displacement_coefficient_x = cos(theta_previous) * data.dt;
            displacement_coefficient_y = sin(theta_previous) * data.dt;
        end
        motion_direction = [displacement_coefficient_x; ...
            displacement_coefficient_y];

        posterior_displacement = ...
            data.leader_posterior.state(1:2,j) ...
            - data.leader_posterior.state(1:2,j-1);

        % MAT 中未保存相邻时刻估计误差的时间交叉协方差，因此这里使用
        % P_pp(j)+P_pp(j-1) 作为保守近似，并明确只把结果用于动画诊断。
        displacement_covariance = ...
            data.leader_posterior.covariance(1:2,1:2,j) ...
            + data.leader_posterior.covariance(1:2,1:2,j-1);
        displacement_covariance = 0.5 * (...
            displacement_covariance + displacement_covariance.');

        covariance_xx = max(displacement_covariance(1,1), 1e-12);
        covariance_yy = max(displacement_covariance(2,2), 1e-12);
        covariance_xy = displacement_covariance(1,2);
        covariance_determinant = ...
            covariance_xx * covariance_yy - covariance_xy^2;
        if covariance_determinant <= 1e-18
            continue;
        end
        inverse_displacement_covariance = (1 / covariance_determinant) * ...
            [covariance_yy, -covariance_xy; ...
            -covariance_xy, covariance_xx];

        interval_information = motion_direction.' ...
            * inverse_displacement_covariance * motion_direction;
        interval_weighted_displacement = motion_direction.' ...
            * inverse_displacement_covariance * posterior_displacement;
        information_sum = information_sum + interval_information;
        weighted_displacement_sum = weighted_displacement_sum ...
            + interval_weighted_displacement;
    end

    if information_sum > 1e-12
        velocity_observation.mean(k) = ...
            weighted_displacement_sum / information_sum;
        velocity_observation.variance(k) = 1 / information_sum;
        velocity_observation.is_available(k) = ...
            isfinite(velocity_observation.mean(k)) ...
            && isfinite(velocity_observation.variance(k)) ...
            && velocity_observation.variance(k) > 0;
    end
end
end

function plot_robot_orientation(axes_handle, x, y, theta, color, linewidth)
%PLOT_ROBOT_ORIENTATION 绘制当前载具朝向，不加入图例。
arrow_length = 1.5;
plot(axes_handle, [x, x + arrow_length * cos(theta)], ...
    [y, y + arrow_length * sin(theta)], color, ...
    'LineWidth', linewidth, 'HandleVisibility', 'off');
end
