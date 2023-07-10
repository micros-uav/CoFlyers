function [states,dstates] = pm_dynamics_update(t, states,commands_bottom,sample_time_motion)
%PM_DYNAMICS_UPDATE Summary of this function goes here
%   Detailed explanation goes here


file_name_param = 'point_mass_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[a_max,...
v_max,...
T_p,...
T_v] = fun_params();

axyz = commands_bottom;

axyz_norm = sqrt(sum(axyz.^2,1));
% Clamp acceleration
clamp_a = find(axyz_norm>a_max);
if ~isempty(clamp_a)
    axyz(:,clamp_a) = axyz(:,clamp_a)./axyz_norm(clamp_a) * a_max;
end

%%% Numerical intergration, first-order
dstates = [states(4:6,:);axyz];
states = states + dstates * sample_time_motion;
% Clamp velocity
vxyz_norm = sqrt(sum(states(4:6,:).^2,1));
clamp_v = find(vxyz_norm >  v_max);
if ~isempty(clamp_v)
    states(4:6,clamp_v) = states(4:6,clamp_v)./vxyz_norm(clamp_v) * v_max;
end

end

