function x_opt = run_auto_tuning(flag_use_parallel,repeat_times,...
    lower_boudary_s, upper_boudary_s, param_name_s, alg_opt_name)
%RUN_AUTO_TUNING Summary of this function goes here
%   Detailed explanation goes here

mode_simulation = 1;
x_num = length(lower_boudary_s);

%%% Save parameters_optimize and values to txt file
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
optimize_target_txt_name = [data_save_dir_name,'/Optimize_',time_now_string];

%%% Handle of optimization function
fun = @(x)model_swarm_repeat(mode_simulation,...
    struct("param_name_s",param_name_s,"param_value_s",{arrayfun(@(xx)string(xx),x)}),...
    [],repeat_times,optimize_target_txt_name,false); 

x_opt = [];
switch alg_opt_name
    case "GA"
        %%% Perform optimization with GA
        options = optimoptions('ga','Display','iter','UseParallel',flag_use_parallel,'PlotFcn', @gaplotbestf,'FunctionTolerance',0.001);
        x_opt = ga(fun,x_num,[],[],[],[],lower_boudary_s,upper_boudary_s,[],options);
    case "PSO"
        options = optimoptions('particleswarm','Display','iter','UseParallel',flag_use_parallel,'PlotFcn', @gaplotbestf,'FunctionTolerance',0.001);
        x_opt = particleswarm(fun,x_num,lower_boudary_s,upper_boudary_s,options);
    otherwise

end
disp(x_opt)

%%% Save data
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
save([data_save_dir_name,'/result_',time_now_string,'.mat']);
end

