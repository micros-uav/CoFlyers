function [values, values_for_visual] = evaluation_module_one(states,parameters_evalue,map_grid,parameters_map,flag_evalue)
%EVALUATION_MODEL_ONE 
[parameters_evalue_sub] = evaluation_module_parameters_deal(parameters_evalue,flag_evalue);

switch flag_evalue
    case 0
        [values, values_for_visual] = evaluation_0_module_one(states,parameters_evalue_sub,map_grid,parameters_map);
    case 1
        [values, values_for_visual] = evaluation_1_module_one(states,parameters_evalue_sub);

end

end