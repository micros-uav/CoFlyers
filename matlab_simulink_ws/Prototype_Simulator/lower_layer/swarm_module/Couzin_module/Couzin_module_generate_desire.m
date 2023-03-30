function [command_upper_s,control_mode_s] =...
    Couzin_module_generate_desire(t,states,parameters_flocking,map_lines,map_grid,parameters_sensor_env)
%COUNZIN_MODULE_GENERATE_DESIRE_I  Generate the desired position and velocity
% according to the current flocking states and model parameters
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];


number = size(states,2);
posDesired = [states(1:3,:);zeros(1,size(states,2))];
velDesired = zeros(4,size(states,2));
accDesired = zeros(4,size(states,2));
control_mode_s = uint8(zeros(1,number))+0b001010;

[alpha,...
    dim,...
    v_flock,...
    r_rep,...
    dro,...
    dra,...
    sigma,...
    num,...
    heights] = Couzin_module_parameters_deal(parameters_flocking);
r_com = r_rep+dro+dra;

for id  = 1:number
    state_i = states(:,id);
    %%%%%Find neighbors%%%%%
    posijArray = repmat(state_i(1:dim),1,number) - states(1:dim,:);
    disij = sqrt(sum(posijArray.^2,1));
    disij(id) = inf;
    %%% Metric
    ind_s_metric = Metric_selection(disij,r_com);
    posid_to_neighbor = posijArray(:,ind_s_metric);
    dis_to_neighbor = disij(ind_s_metric);
    states_neighbor = states(:,ind_s_metric);
    %%% FOV
    ind_s_fov      = FOV_selection(posid_to_neighbor./dis_to_neighbor,state_i(4:3+dim)./norm(state_i(4:3+dim)),alpha);
    posid_to_neighbor = posid_to_neighbor(:,ind_s_fov);
    dis_to_neighbor = dis_to_neighbor(ind_s_fov);
    states_neighbor = states_neighbor(:,ind_s_fov);

    %%%%%Get sensor datas from map_lines
%     datas_sensor_env = sensor_env_module_get_data_from_map_lines(state_i(1:3),0,map_lines,parameters_sensor_env);
    datas_sensor_env = [];
    %%%%%Distributed control%%%%%
    [posDesired_id,velDesired_id,accDesired_id,control_mode_id] =...
            Couzin_module_generate_desire_i(id,state_i,states_neighbor,...
            dis_to_neighbor,posid_to_neighbor,parameters_flocking,map_lines,map_grid,datas_sensor_env);
    

    %%%%%  %%%%%
    posDesired(:,id) = posDesired_id;
    velDesired(:,id) = velDesired_id;
    accDesired(:,id) = accDesired_id;
    control_mode_s(id) = control_mode_id;
end
command_upper_s = [posDesired;velDesired;accDesired];

end

