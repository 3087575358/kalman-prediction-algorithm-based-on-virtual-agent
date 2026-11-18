function project_root = setup_kalman_prediction_project_path()
%SETUP_KALMAN_PREDICTION_PROJECT_PATH 设置项目路径，便于调试和打包。
% 直接运行主脚本时并不强制调用本函数；主脚本也会自动加入 auxiliary。

project_root = fileparts(mfilename('fullpath'));
auxiliary_folder = fullfile(project_root, 'auxiliary');

if ~isfolder(auxiliary_folder)
    error('setup_kalman_prediction_project_path:MissingAuxiliaryFolder', ...
        '没有找到 auxiliary 文件夹：%s', auxiliary_folder);
end

addpath(project_root);
addpath(auxiliary_folder);
fprintf('已加入卡尔曼预测项目路径：%s\n', project_root);
end
