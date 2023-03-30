function parameters_evalue = evaluation_module_parameters(v_flock, flag_evalue, p_bp)
%EVALUATION_MODEL_PARAMETERS 
%   

global_params = [];
 
switch flag_evalue
    case 0
        parameters_evalue_sub = evaluation_0_module_parameters(v_flock);
    case 1
        parameters_evalue_sub = evaluation_1_module_parameters();

    otherwise
        parameters_evalue_sub = [];
end

parameters_evalue = [global_params;parameters_evalue_sub];


% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_evalue(p_bp(1,k)) = p_bp(2,k);
    end
end
end

