function parameters_swarm = swarm_module_parameters(number_h,map_lines,flag_alg,p_bp,P_op)
%MODEL_PARAMETERS 

global_params = [];

switch flag_alg
    case 0
        parameters_swarm = Vasarhelyi_module_parameters(number_h,map_lines);
    case 1
        parameters_swarm = Vasarhelyi_will_module_parameters(number_h,map_lines);
    case 2
        parameters_swarm =Couzin_module_parameters(number_h);

    otherwise
        parameters_swarm = [];
end

parameters_swarm = [global_params;parameters_swarm];

%%% Modify parameters to perform parameter auto-tuning
for i = 1:size(P_op,2)
    if i >length(parameters_swarm)
        break
    end
    parameters_swarm(P_op(1,i)) = P_op(2,i);
end

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_swarm(p_bp(1,k)) = p_bp(2,k);
    end
end

end

