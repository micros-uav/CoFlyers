function values = evaluation_module_average(values_series,parameters_evalue, flag_evalue)
%EVALUATION_MODEL_ONE 
%   
[parameters_evalue_sub] = evaluation_module_parameters_deal(parameters_evalue,flag_evalue);

switch flag_evalue
    case 0
        [values] = evaluation_0_module_average(values_series,parameters_evalue_sub);
    case 1
        [values] = evaluation_1_module_average(values_series,parameters_evalue_sub);

end

end