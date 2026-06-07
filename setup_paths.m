function project_root = setup_paths()
%SETUP_PATHS 配置 research 项目的 MATLAB 搜索路径。
%   所有脚本仍以项目根目录为运行基准；本函数返回项目根目录绝对路径。

project_root = fileparts(mfilename('fullpath'));

path_dirs = {
    project_root
    fullfile(project_root, 'apps')
    fullfile(project_root, 'analysis')
    fullfile(project_root, 'experiments')
    fullfile(project_root, 'experiments', 'diagnostics')
    fullfile(project_root, 'tests', 'integration')
    fullfile(project_root, 'tests', 'model')
    fullfile(project_root, 'tests', 'eso')
    fullfile(project_root, 'src', 'matlab')
    fullfile(project_root, 'src', 'matlab', 'Lib')
    fullfile(project_root, 'src', 'matlab', 'guidance')
    fullfile(project_root, 'src', 'matlab', 'controller', 'xhy')
    fullfile(project_root, 'src', 'matlab', 'controller', 'remus')
    fullfile(project_root, 'src', 'matlab', 'model')
    fullfile(project_root, 'src', 'matlab', 'model', 'params')
    fullfile(project_root, 'src', 'matlab', 'eso')
    fullfile(project_root, 'src', 'matlab', 'post')
    fullfile(project_root, 'src', 'matlab', 'traj')
    fullfile(project_root, 'src', 'matlab', 'RL')
    fullfile(project_root, 'src', 'matlab', 'RL', 'env')
    fullfile(project_root, 'src', 'matlab', 'RL', 'lib')
    fullfile(project_root, 'src', 'matlab', 'RL', 'set')
};

for i = 1:numel(path_dirs)
    if exist(path_dirs{i}, 'dir')
        addpath(path_dirs{i});
    end
end

output_dirs = {
    fullfile(project_root, 'results')
    fullfile(project_root, 'assets', 'figures')
    fullfile(project_root, 'paper', 'figures')
};

for i = 1:numel(output_dirs)
    if ~exist(output_dirs{i}, 'dir')
        mkdir(output_dirs{i});
    end
end
end
