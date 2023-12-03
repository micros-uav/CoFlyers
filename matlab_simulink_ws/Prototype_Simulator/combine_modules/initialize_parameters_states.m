function [states,dstates,...
    map3d_faces, map3d_struct, model_stls] = ...
initialize_parameters_states(P_GUI, P_op, P_bp, mode_simulation, xml_name)
% P_GUI: parameters from gui
% P_op:  parameters to be optimized
% P_bp:  parameters for batch processing


ps_mofify = struct("param_name_s",[],"param_value_s",[]);

% GUI Parameters
if ~isempty(P_GUI)
    ps_mofify = P_GUI;
    % field_names1 = fieldnames(P_GUI.parameters_settings);
    % field_names_full1 = arrayfun(@(x)strcat("CoFlyers.",x),field_names1);
    % field_values1 = struct2cell(P_GUI.parameters_settings);
    % field_names2 = fieldnames(P_GUI.parameters_visual);
    % field_names_full2 = arrayfun(@(x)strcat("CoFlyers.visual.",x),field_names2);
    % field_values2 = struct2cell(P_GUI.parameters_visual);
    % ps_mofify.param_name_s = [field_names_full1',field_names_full2'];
    % ps_mofify.param_value_s = [field_values1',field_values2'];
end
% Batch processing parameters
if ~isempty(P_bp)
    ps_mofify = P_bp;
end
% Parameter auto-tuning parameters
if ~isempty(P_op)
    ps_mofify = P_op;
end

[map3d_faces, map3d_struct, model_stls, ~, position0] = read_parameter_xml(xml_name, ps_mofify);


[number,...
time_max,...
sample_time_motion,...
sample_time_control_upper,...
sample_time_control_bottom,...
activate_save_states,...
time_interval_save,...
motion_model_type,...
swarm_algorithm_type,...
evaluation_metric_type] = setting_parameters();


%%% Initialize state

[states,dstates] = initialize_states(position0, motion_model_type);

%%% Find out the parameters of the mth module for batch processing
    function p_bp = find_out_params_of_mth_module_for_bp(P_bp,m)
        if ~isempty(P_bp)
            if size(P_bp,1) ~= 3
                p_bp = [];
                disp('Per column of P_bp must have 3 elements: m, n, value.');
                return
            end
            %
            temp = find(P_bp(1,:) == m);
            if ~isempty(temp)
                p_bp = P_bp(2:3,temp);
            else
                p_bp = [];
            end
        else
            p_bp = [];
        end
    end
end