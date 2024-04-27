clc;
close all;
mode_simulation       = 0; % 0 Running test, 1 Auto-tuning, 2 Patch processing
flag_app = false;
if exist('app','var')
    if ~isempty(app)
        flag_app = true;
    end
end
if ~flag_app
    app = [];
    parameters_gui = [];
%     xml_name = 'xml_config_files\parameters.xml';
    % xml_name = 'xml_config_files\parameters_vicsek18_64.xml';
    % xml_name = 'xml_config_files\parameters_PSO.xml';
    % xml_name = 'xml_config_files\parameters_vicsek95_h_d_l_n.xml';
    % xml_name = 'xml_config_files\parameters_viscek95_l_d_l_n.xml';
    % xml_name = 'xml_config_files\parameters_viscek95_h_d_h_n.xml';
    % xml_name = 'xml_config_files\parameters_vicsek95_h_d_l_n.xml';
    % xml_name = 'xml_config_files\parameters_couzin2002_dynamic_highly_parallel.xml';
    % xml_name = 'xml_config_files\parameters_couzin2002_dynamic_parallel.xml';
    % xml_name = 'xml_config_files\parameters_couzin2002_swarm.xml';
    % xml_name = 'xml_config_files\parameters_couzin2002_torus.xml';
    % xml_name = 'xml_config_files\parameters_obstacles.xml';
    xml_name = 'xml_config_files\parameters_terrain.xml';
else
    parameters_gui = app.get_all_param();
    xml_name = get_xml_name(app);
end
parameters_auto_tuning = [];
parameters_batch_processing    = [];

% ts = tic;
% repeat_num = 1;
% for i = 1:repeat_num
tic
values = model_swarm(parameters_gui, parameters_auto_tuning,...
    parameters_batch_processing, 1, mode_simulation, xml_name, app);
disp(values');
toc

% end           
% time_real_one = toc(ts)/repeat_num;
% % save("aaa.mat","time_real_one");
% disp(['The elapsed time is ',num2str(time_real_one,'%.04f'),' s']);