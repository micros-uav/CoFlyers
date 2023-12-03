function results = run_batch_processing(flag_use_parallel, type_parallel,...
    repeat_times, parameters_bp_s, xml_name)
%RUN_BATCH_PROCESSING Summary of this function goes here
%   Detailed explanation goes here
mode_simulation = 2;

%%%
results = [];

%%% Traverse
if flag_use_parallel
    if type_parallel == 1
        parfor k = 1:length(parameters_bp_s)
            parameters_bp = parameters_bp_s(k);
            results(:,k) = model_swarm_repeat(mode_simulation, [],parameters_bp,repeat_times,[],false,xml_name);
            disp(['No.',num2str(k),': ',num2str(results(:,k)')])
        end
    else
        for k = 1:length(parameters_bp_s)
            parameters_bp = parameters_bp_s(k);
            results(:,k) = model_swarm_repeat(mode_simulation, [],parameters_bp,repeat_times,[],true,xml_name);
            disp(['No.',num2str(k),': ',num2str(results(:,k)')])
        end
    end
else
    for k = 1:length(parameters_bp_s)
        parameters_bp = parameters_bp_s(k);
        results(:,k) = model_swarm_repeat(mode_simulation, [],parameters_bp,repeat_times,[],false,xml_name); 
        disp(['No.',num2str(k),': ',num2str(results(:,k)')])
    end
end

%%% Save data
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
save([data_save_dir_name,'/results_',time_now_string,'.mat']);

end

