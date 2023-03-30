function values = evaluation_1_module_average(values_series,parameters_evalue)
%EVALUATION_MODEL_ONE 
%   
evaluation_1_module_parameters_deal(parameters_evalue);

p_group = mean(values_series(1,:));
m_gourp  = mean(values_series(2,:));

F = m_gourp;

values = [F;p_group;m_gourp];
end