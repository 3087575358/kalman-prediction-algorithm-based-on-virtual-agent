%% 比较完整协方差 Kalman 与强制 Pxy=0 的对照实验
% 自动读取 saves 中最新的两组结果，生成 MAT 数据表、CSV 和时间对比图。

clear; close all; clc;

script_full_path = mfilename('fullpath');
project_root = fileparts(script_full_path);
saves_folder = fullfile(project_root, 'saves');

baseline_candidates = dir(fullfile(saves_folder, 'kalman-test-*.mat'));
baseline_candidates = baseline_candidates(~contains({baseline_candidates.name}, 'pxy-zero'));
pxy_zero_candidates = dir(fullfile(saves_folder, 'kalman-test-pxy-zero-*.mat'));

if isempty(baseline_candidates)
    error('没有找到原版仿真 MAT 文件。');
end
if isempty(pxy_zero_candidates)
    error('没有找到 Pxy=0 对照仿真 MAT 文件。');
end

[~, baseline_index] = max([baseline_candidates.datenum]);
[~, pxy_zero_index] = max([pxy_zero_candidates.datenum]);
baseline_file = fullfile(saves_folder, baseline_candidates(baseline_index).name);
pxy_zero_file = fullfile(saves_folder, pxy_zero_candidates(pxy_zero_index).name);

baseline = load(baseline_file);
pxy_zero = load(pxy_zero_file);

if length(baseline.t) ~= length(pxy_zero.t) || any(abs(baseline.t - pxy_zero.t) > 1e-12)
    error('两次仿真的时间轴不一致，不能直接比较。');
end

time_s = baseline.t(:);
sample_count = length(time_s);
% 原主循环在最后一次迭代仍写入 leader(k+1)，因此 leader 轨迹比 t 多一个元素。
% 对比表严格截取与 t 对应的 1:N，不改变原程序时序。
true_x_m = baseline.leader.x(1:sample_count).';
true_y_m = baseline.leader.y(1:sample_count).';

baseline_prior_x_m = baseline.leader_prior.x(:);
baseline_prior_y_m = baseline.leader_prior.y(:);
pxy_zero_prior_x_m = pxy_zero.leader_prior.x(:);
pxy_zero_prior_y_m = pxy_zero.leader_prior.y(:);

baseline_posterior_x_m = baseline.leader_posterior.x(:);
baseline_posterior_y_m = baseline.leader_posterior.y(:);
pxy_zero_posterior_x_m = pxy_zero.leader_posterior.x(:);
pxy_zero_posterior_y_m = pxy_zero.leader_posterior.y(:);

baseline_prior_error_x_m = baseline_prior_x_m - true_x_m;
baseline_prior_error_y_m = baseline_prior_y_m - true_y_m;
pxy_zero_prior_error_x_m = pxy_zero_prior_x_m - true_x_m;
pxy_zero_prior_error_y_m = pxy_zero_prior_y_m - true_y_m;

baseline_posterior_error_x_m = baseline_posterior_x_m - true_x_m;
baseline_posterior_error_y_m = baseline_posterior_y_m - true_y_m;
pxy_zero_posterior_error_x_m = pxy_zero_posterior_x_m - true_x_m;
pxy_zero_posterior_error_y_m = pxy_zero_posterior_y_m - true_y_m;

baseline_prior_position_error_m = hypot(baseline_prior_error_x_m, baseline_prior_error_y_m);
pxy_zero_prior_position_error_m = hypot(pxy_zero_prior_error_x_m, pxy_zero_prior_error_y_m);
baseline_posterior_position_error_m = hypot(baseline_posterior_error_x_m, baseline_posterior_error_y_m);
pxy_zero_posterior_position_error_m = hypot(pxy_zero_posterior_error_x_m, pxy_zero_posterior_error_y_m);

baseline_prior_pxy = reshape(baseline.leader_prior.covariance(1,2,:), [], 1);
pxy_zero_prior_pxy = reshape(pxy_zero.leader_prior.covariance(1,2,:), [], 1);
baseline_posterior_pxy = reshape(baseline.leader_posterior.covariance(1,2,:), [], 1);
pxy_zero_posterior_pxy = reshape(pxy_zero.leader_posterior.covariance(1,2,:), [], 1);

comparison_table = table(time_s, true_x_m, true_y_m, ...
    baseline_prior_x_m, baseline_prior_y_m, ...
    pxy_zero_prior_x_m, pxy_zero_prior_y_m, ...
    baseline_posterior_x_m, baseline_posterior_y_m, ...
    pxy_zero_posterior_x_m, pxy_zero_posterior_y_m, ...
    baseline_prior_error_x_m, baseline_prior_error_y_m, ...
    pxy_zero_prior_error_x_m, pxy_zero_prior_error_y_m, ...
    baseline_prior_position_error_m, pxy_zero_prior_position_error_m, ...
    baseline_posterior_position_error_m, pxy_zero_posterior_position_error_m, ...
    baseline_prior_pxy, pxy_zero_prior_pxy, ...
    baseline_posterior_pxy, pxy_zero_posterior_pxy);

method = {'完整协方差'; '强制 Pxy=0'};
prior_rmse_x_m = [sqrt(mean(baseline_prior_error_x_m.^2)); ...
    sqrt(mean(pxy_zero_prior_error_x_m.^2))];
prior_rmse_y_m = [sqrt(mean(baseline_prior_error_y_m.^2)); ...
    sqrt(mean(pxy_zero_prior_error_y_m.^2))];
prior_rmse_position_m = [sqrt(mean(baseline_prior_position_error_m.^2)); ...
    sqrt(mean(pxy_zero_prior_position_error_m.^2))];
posterior_rmse_position_m = [sqrt(mean(baseline_posterior_position_error_m.^2)); ...
    sqrt(mean(pxy_zero_posterior_position_error_m.^2))];
prior_max_position_error_m = [max(baseline_prior_position_error_m); ...
    max(pxy_zero_prior_position_error_m)];
summary_table = table(method, prior_rmse_x_m, prior_rmse_y_m, ...
    prior_rmse_position_m, posterior_rmse_position_m, prior_max_position_error_m);

timestamp = datestr(now, 'yy-mm-dd-HHMMSS');
comparison_mat_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-comparison-%s.mat', timestamp));
comparison_csv_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-comparison-%s.csv', timestamp));
summary_csv_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-comparison-summary-%s.csv', timestamp));

source_baseline_mat = baseline_file;
source_pxy_zero_mat = pxy_zero_file;
save(comparison_mat_file, 'comparison_table', 'summary_table', ...
    'source_baseline_mat', 'source_pxy_zero_mat');
writetable(comparison_table, comparison_csv_file);
writetable(summary_table, summary_csv_file);

trajectory_figure = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1400, 850]);

subplot(2,2,1);
plot(time_s, baseline_prior_x_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, true_x_m, 'b-', 'LineWidth', 1.0);
grid on; xlabel('时间 (s)'); ylabel('X 位置 (m)');
title('完整协方差：X 预测值与真实值');
legend('Kalman 先验预测值', '真实值', 'Location', 'best');

subplot(2,2,2);
plot(time_s, pxy_zero_prior_x_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, true_x_m, 'b-', 'LineWidth', 1.0);
grid on; xlabel('时间 (s)'); ylabel('X 位置 (m)');
title('强制 P_{xy}=0：X 预测值与真实值');
legend('Kalman 先验预测值', '真实值', 'Location', 'best');

subplot(2,2,3);
plot(time_s, baseline_prior_y_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, true_y_m, 'b-', 'LineWidth', 1.0);
grid on; xlabel('时间 (s)'); ylabel('Y 位置 (m)');
title('完整协方差：Y 预测值与真实值');
legend('Kalman 先验预测值', '真实值', 'Location', 'best');

subplot(2,2,4);
plot(time_s, pxy_zero_prior_y_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, true_y_m, 'b-', 'LineWidth', 1.0);
grid on; xlabel('时间 (s)'); ylabel('Y 位置 (m)');
title('强制 P_{xy}=0：Y 预测值与真实值');
legend('Kalman 先验预测值', '真实值', 'Location', 'best');

trajectory_png_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-prediction-vs-truth-%s.png', timestamp));
trajectory_fig_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-prediction-vs-truth-%s.fig', timestamp));
print(trajectory_figure, trajectory_png_file, '-dpng', '-r180');
savefig(trajectory_figure, trajectory_fig_file);
close(trajectory_figure);

error_figure = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1300, 760]);

subplot(2,1,1);
plot(time_s, baseline_prior_position_error_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, pxy_zero_prior_position_error_m, 'b-', 'LineWidth', 1.2);
grid on; xlabel('时间 (s)'); ylabel('先验位置误差 (m)');
title('Kalman 先验预测位置误差对比');
legend('完整协方差', '强制 P_{xy}=0', 'Location', 'best');

subplot(2,1,2);
plot(time_s, baseline_posterior_position_error_m, 'r-', 'LineWidth', 1.2); hold on;
plot(time_s, pxy_zero_posterior_position_error_m, 'b-', 'LineWidth', 1.2);
grid on; xlabel('时间 (s)'); ylabel('后验位置误差 (m)');
title('Kalman 融合后位置误差对比');
legend('完整协方差', '强制 P_{xy}=0', 'Location', 'best');

error_png_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-error-comparison-%s.png', timestamp));
error_fig_file = fullfile(saves_folder, ...
    sprintf('kalman-pxy-error-comparison-%s.fig', timestamp));
print(error_figure, error_png_file, '-dpng', '-r180');
savefig(error_figure, error_fig_file);
close(error_figure);

fprintf('原版 MAT：%s\n', baseline_file);
fprintf('Pxy=0 MAT：%s\n', pxy_zero_file);
fprintf('对比数据 MAT：%s\n', comparison_mat_file);
fprintf('对比数据 CSV：%s\n', comparison_csv_file);
fprintf('汇总 CSV：%s\n', summary_csv_file);
fprintf('预测值/真实值图：%s\n', trajectory_png_file);
fprintf('误差对比图：%s\n', error_png_file);
disp(summary_table);
