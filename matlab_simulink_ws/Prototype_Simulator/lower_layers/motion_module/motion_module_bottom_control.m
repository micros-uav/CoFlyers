function commands_bottom = motion_module_bottom_control(t, states,command_upper_s,...
    control_mode_s_u, flag_motion_model,sample_time_control_b)
%MOTION_MODULE_BOTTOM_CONTROL Summary of this function goes here

number = size(states,2);
control_mode_unique_u = unique(control_mode_s_u);

%%% Get the dimension of commands_bottom
dim = motion_module_get_dim_commands_bottom(flag_motion_model);
commands_bottom = zeros(dim,number);

% control_mode_s = control_mode_s_u;


%%% Calculate commands_bottom
for i  = 1:length(control_mode_unique_u)
    cm = control_mode_unique_u(i);
    ind_s = find(control_mode_s_u ==cm);
    switch flag_motion_model
        case 0 % point-mass
            control_mode_s_b = pm_mode_upper2bottom(control_mode_s_u(ind_s));
            commands_bottom(:,ind_s) = pm_control(t, states(:,ind_s),...
                command_upper_s(1:3,ind_s), command_upper_s(5:7,ind_s),command_upper_s(9:11,ind_s),...
                control_mode_s_b,sample_time_control_b);
        case 1 % quadcopter
            control_mode_s_b = qm_mode_upper2bottom(control_mode_s_u(ind_s));
            commands_bottom(:,ind_s) = qm_control(t, states(:,ind_s),...
                command_upper_s(1:4,ind_s), command_upper_s(5:8,ind_s),command_upper_s(9:12,ind_s), 0,...
                control_mode_s_b,sample_time_control_b);
        case 2 % point-mass rotation
            control_mode_s_b = pm_mode_upper2bottom(control_mode_s_u(ind_s));
            commands_bottom(:,ind_s) = pmr_control(t, states(:,ind_s),...
                command_upper_s(1:3,ind_s), command_upper_s(5:7,ind_s),...
                control_mode_s_b,sample_time_control_b);
    end
end

end

