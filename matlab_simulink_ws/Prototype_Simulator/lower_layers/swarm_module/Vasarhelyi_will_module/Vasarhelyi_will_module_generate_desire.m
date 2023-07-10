function [command_upper_s,control_mode_s,will_s_for_visual] =...
    Vasarhelyi_will_module_generate_desire(t,states, sample_time, sensor_data_s)
%VASARHELYI_WILL_MODULE_GENERATE_DESIRE_I  Generate the desired position and velocity
% according to the current flocking states and model parameters
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];



number = size(states,2);
persistent will_s
if isempty(will_s)
    will_s = zeros(1,number);
end
if t==0 
    will_s = zeros(1,number);
end


posDesired = [states(1:3,:);zeros(1,size(states,2))];
velDesired = zeros(4,size(states,2));
accDesired = zeros(4,size(states,2));
control_mode_s = uint8(zeros(1,number)) + 7;

file_name_param = 'Vasarhelyi_will_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[r_com,~,tau_will] = fun_params();

for id  = 1:number
    state_i = states(:,id);
    will_id = will_s(id);

    %%%%%Find neighbors%%%%%
    posId = state_i(1:2); 
    posijArray = repmat(posId,1,number) - states(1:2,:);
    disij = sqrt(sum(posijArray.^2,1));
    disij(id) = inf;
    %%% Metric
    ind_s_metric = Metric_selection(disij,r_com);
    posid_to_neighbor = posijArray(:,ind_s_metric);
    dis_to_neighbor = disij(ind_s_metric);
    states_neighbor = states(:,ind_s_metric);
    will_neighbor_s = will_s(ind_s_metric);
%     will_neighbor_s = will_neighbor_s * 0.7;
    %%%%%Get sensor datas from map_lines
%     datas_sensor_env = sensor_env_module_get_data_from_map_lines(state_i(1:3),0,map_lines,parameters_sensor_env);
%     datas_sensor_env = [];
    %%%%%Distributed control%%%%%
    [posDesired_id,velDesired_id,accDesired_id,control_mode_id,will_id] =...
            Vasarhelyi_will_module_generate_desire_i(id,state_i,states_neighbor,...
            dis_to_neighbor,posid_to_neighbor,...
            will_id, will_neighbor_s);
    will_s(id) = will_id;

    %%%%%  %%%%%
    posDesired(:,id) = posDesired_id;
    velDesired(:,id) = velDesired_id;
    accDesired(:,id) = accDesired_id;
    control_mode_s(id) = control_mode_id;
    
end
will_s = will_s * exp(-sample_time/tau_will);

command_upper_s = [posDesired;velDesired;accDesired];
will_s_for_visual = will_s;
end

