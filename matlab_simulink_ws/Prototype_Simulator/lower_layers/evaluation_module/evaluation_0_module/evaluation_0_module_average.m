function values = evaluation_0_module_average(time_series, values_series)
%EVALUATION_MODEL_ONE 

v_tol = 0;
r_tol = 0;

file_name_param = 'evaluation_0_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[v_flock,...
r_coll,...
a_tol] = fun_params();

phiCorr = mean(values_series(1,:));
phiVel  = mean(values_series(2,:));
phiColl = mean(values_series(3,:));
phiWall = mean(values_series(4,:));
phiMND = mean(values_series(5,:));

[~,~,~,~,F] = fitness_function_combine(v_flock,a_tol,v_tol,r_tol,phiVel, ...
    phiColl,phiWall,phiCorr);

values = [F;phiCorr;phiVel;phiColl;phiWall;phiMND];
end