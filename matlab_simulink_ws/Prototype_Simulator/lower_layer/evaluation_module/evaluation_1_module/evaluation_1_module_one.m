function [values, values_for_visual] = evaluation_1_module_one(states,parameters_evalue)
%EVALUATION_MODEL_ONE 
% 
evaluation_1_module_parameters_deal(parameters_evalue);

number = size(states,2); 
position = states(1:3,:);
velocity = states(4:6,:);
velocity_uint = velocity./vecnorm(velocity);

center = mean(position,2);
position_rel = position - center;
position_rel_unit = position_rel./vecnorm(position_rel);


p_group = norm(mean(velocity_uint,2));
m_group = norm(mean(cross(position_rel_unit',velocity_uint')));


values = [p_group;m_group];

%%% For visual
values_for_visual = [];
end