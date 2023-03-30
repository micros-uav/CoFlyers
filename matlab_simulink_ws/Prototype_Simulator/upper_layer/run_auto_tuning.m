function x_opt = run_auto_tuning(flag_use_parallel,repeat_times,...
    lower_boudaries, upper_boudaries, order_s, flag_alg_opt)
%RUN_AUTO_TUNING Summary of this function goes here
%   Detailed explanation goes here
% flag_alg_opt: 0-Genetic Algorithm, 1-Particle Swarm Optimization, ...
mode_simulation = 1;
GA = 0;
PSO = 1;

x_num = length(lower_boudaries);

%%% Save parameters_optimize and values to txt file
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
optimize_target_txt_name = [data_save_dir_name,'/Optimize_',time_now_string];

%%% Handle of optimization function
fun = @(x)model_swarm_repeat(mode_simulation,[order_s;x],[],repeat_times,optimize_target_txt_name); 

x_opt = [];
switch flag_alg_opt
    case GA
        %%% Perform optimization with GA
        options = optimoptions('ga','Display','iter','UseParallel',flag_use_parallel,'PlotFcn', @gaplotbestf,'FunctionTolerance',0.001);
        x_opt = ga(fun,x_num,[],[],[],[],lower_boudaries,upper_boudaries,[],options);
    case PSO
        options = optimoptions('particleswarm','Display','iter','UseParallel',flag_use_parallel,'PlotFcn', @gaplotbestf,'FunctionTolerance',0.001);
        x_opt = particleswarm(fun,x_num,lower_boudaries,upper_boudaries,options);
    otherwise

end
disp(x_opt)

%%% Save data
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
save([data_save_dir_name,'/result_',time_now_string,'.mat']);
end

