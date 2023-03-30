function [posDesired_id,velDesired_id,accDesired_id,control_mode_id] = Couzin_module_generate_desire_i(id,state_i,states_neighbor,...
        dis_to_neighbor,posid_to_neighbor,parameters_flocking,map_lines,map_grid,datas_sensor_env)
%VASARHELYI_MODULE_GENERATE_DESIRE_I Summary of this function goes here
%   Detailed explanation goes here
[alpha,...
    dim,...
    v_flock,...
    r_rep,...
    dro,...
    dra,...
    sigma,...
    num,...
    heights] = Couzin_module_parameters_deal(parameters_flocking);
r_ori = r_rep+dro;
r_att = r_rep + dra;
if dim == 2
    posDesired_id = [state_i(1:2);heights(id);0];
else
    posDesired_id = [state_i(1:3);0];
end
velDesired_id = zeros(4,1);
accDesired_id = zeros(4,1);

VELOCITY_CONTROL_TYPE = 6;
VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;
if dim == 2
    control_mode_id = VELOCITY_HORIZONTAL_CONTROL_TYPE;
else
    control_mode_id = VELOCITY_CONTROL_TYPE;
end

if ~isempty(states_neighbor)
    %
    in_rep = dis_to_neighbor<r_rep;

    % Repulsion
    if sum(in_rep)>0
        r_neigh_in_rep = posid_to_neighbor(1:dim,in_rep);
        v_rep = sum(r_neigh_in_rep./vecnorm(r_neigh_in_rep),2);
        v_rep = v_rep./norm(v_rep);
        %%%%%Final equation of desired velocity%%%%%
        velDesired_id(1:dim) = v_rep*v_flock;
    else
        in_ori = dis_to_neighbor>=r_rep & dis_to_neighbor<r_ori;
        in_att = (~in_rep) & (~in_ori);
        % Alignment
        v_ori = zeros(dim,1);
        if sum(in_ori) > 0
            v_neigh_in_ori = states_neighbor(4:3+dim,in_ori);
            v_ori = sum(v_neigh_in_ori./vecnorm(v_neigh_in_ori),2);
        end
        % Attraction
        v_att = zeros(dim,1);
        if sum(in_att)>0
            r_neigh_in_att = posid_to_neighbor(1:dim,in_att);
            v_att = -sum(r_neigh_in_att./vecnorm(r_neigh_in_att),2);
        end
        temp = v_ori + v_att;
        temp = temp/norm(temp);
        %%%%%Final equation of desired velocity%%%%%
        velDesired_id(1:dim) = temp*v_flock;
    end

else
    velDesired_id(1:dim) = state_i(4:3+dim)/norm(state_i(4:3+dim))*v_flock;
end

%%% Add angular noise
alpha = normrnd(0,sigma);
if dim == 2
    cos_alpha = cos(alpha);
    sin_alpha = sin(alpha);
    velDesired_id(1) = cos_alpha.*velDesired_id(1) -   sin_alpha.*velDesired_id(2);
    velDesired_id(2) =  sin_alpha.*velDesired_id(1) + cos_alpha.*velDesired_id(2);
else
    rot_dir = rand(3,1); rot_dir = rot_dir/norm(rot_dir);
    skew_R = [0,-rot_dir(3),rot_dir(2);rot_dir(3),0,-rot_dir(1);-rot_dir(2),rot_dir(1),0];
    R = cos(alpha)*eye(3) + (1-cos(alpha))*(rot_dir*rot_dir') + sin(alpha)*skew_R;
    velDesired_id(1:3) = R*velDesired_id(1:3);
end

end

