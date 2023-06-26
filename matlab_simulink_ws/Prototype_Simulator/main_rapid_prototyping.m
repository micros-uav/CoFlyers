clc;
close all;
mode_simulation       = 0; % 0 Running test, 1 Auto-tuning, 2 Patch processing
if ~exist('app','var')
    app = [];
end
parameters_gui = get_params_from_gui(app);
parameters_auto_tuning = [];
parameters_batch_processing    = [];

% ts = tic;
% repeat_num = 1;
% for i = 1:repeat_num
tic
values = model_swarm(parameters_gui, parameters_auto_tuning, parameters_batch_processing,1, mode_simulation);
disp(values');
toc

% end
% time_real_one = toc(ts)/repeat_num;
% % save("aaa.mat","time_real_one");
% disp(['The elapsed time is ',num2str(time_real_one,'%.04f'),' s']);