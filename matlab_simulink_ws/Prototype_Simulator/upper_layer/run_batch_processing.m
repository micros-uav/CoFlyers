function results = run_batch_processing(flag_use_parallel,...
    repeat_times, parameters_bp_s)
%RUN_BATCH_PROCESSING Summary of this function goes here
%   Detailed explanation goes here
mode_simulation = 2;

%%%
results = [];

%%% Traverse
if flag_use_parallel
    parfor k = 1:size(parameters_bp_s,3)
        parameters_bp = parameters_bp_s(:,:,k);
        results(:,k) = model_swarm_repeat(mode_simulation, [],parameters_bp,repeat_times,[]); 
        disp(['No.',num2str(k),': ',num2str(results(:,k)')])
    end
else
    for k = 1:size(parameters_bp_s,3)
        parameters_bp = parameters_bp_s(:,:,k);
        results(:,k) = model_swarm_repeat(mode_simulation, [],parameters_bp,repeat_times,[]); 
        disp(['No.',num2str(k),': ',num2str(results(:,k)')])
    end
end

%%% Save data
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
save([data_save_dir_name,'/results_',time_now_string,'.mat']);

end
