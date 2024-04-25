function [posDesired_id,velDesired_id,accDesired_id,control_mode_id] = Vasarhelyi_module_generate_desire_i(id,state_i,states_neighbor,...
        dis_to_neighbor,posid_to_neighbor,terrain,terrain_params)
%VASARHELYI_MODULE_GENERATE_DESIRE_I Summary of this function goes here
%   Detailed explanation goes here

file_name_param = 'Vasarhelyi_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[r_com,...
v_flock,...
r_rep_0,...
p_rep,...
r_frict_0,...
c_frict,...
v_frict,...
p_frict,...
a_frict,...
r_shill_0,...
v_shill,...
p_shill,...
a_shill,...
v_max,...
dim,...
height,...
dr_shill,...
pos_shill,...
vel_shill] = fun_params();




VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;

posDesired_id = [state_i(1:2);height;0];
velDesired_id = zeros(4,1);
accDesired_id = zeros(4,1);
control_mode_id = VELOCITY_HORIZONTAL_CONTROL_TYPE;

pos2DId = state_i(1:2);
vel2DId = state_i(4:5);
vel2D_neighbor = states_neighbor(4:5,:);

% Local terrain to shill agents
if ~isempty(terrain)
    r_w = 5;
    r_sub = floor((pos2DId(2)-terrain_params(2,1))/terrain_params(2,2));
    c_sub = floor((pos2DId(1)-terrain_params(1,1))/terrain_params(1,2));
    h_sub = floor((r_w/terrain_params(2,2)));
    w_sub = floor((r_w/terrain_params(1,2)));
    [h,w] = size(terrain);
    r_min = max(1,r_sub-h_sub);
    r_max = min(h,r_sub+h_sub);
    c_min = max(1,c_sub-w_sub);
    c_max = min(w,c_sub+w_sub);
    terrain_sub = terrain(r_min:r_max,c_min:c_max);
    [r_obs,c_obs] = find(terrain_sub>state_i(3));
    if ~isempty(r_obs)
        r_obs = r_obs + r_min - 1;
        c_obs = c_obs + c_min - 1;
        temp_p_shill = [(c_obs'*terrain_params(1,2))+terrain_params(1,1)
            (r_obs'*terrain_params(2,2))+terrain_params(2,1)];
        temp = [pos2DId - temp_p_shill];
        vel_shill = [vel_shill, temp./vecnorm(temp)];
        pos_shill = [pos_shill, temp_p_shill];
    end
end

%Self-propelling term
velIdNorm = norm(vel2DId);
if velIdNorm == 0
    velRandom = rand(dim,1); velRandom = velRandom/norm(velRandom);
    vFlockId = v_flock * velRandom;
else
    vFlockId = v_flock * vel2DId/velIdNorm;
end
%    vFlockId = v_flock * [1;0];


vRepId = zeros(2,1);
vFrictId = zeros(2,1);
if ~isempty(dis_to_neighbor)
    %%%%%Repulsion term%%%%%
    inRepInCom = find(dis_to_neighbor<r_rep_0);
    vRepId = zeros(dim,1);
    if ~isempty(inRepInCom)
        disijInRep = repmat(dis_to_neighbor(inRepInCom),dim,1);
        vRepId = p_rep * sum((r_rep_0 - disijInRep) .* posid_to_neighbor(:,inRepInCom)./disijInRep,2);
%         vRepId = [1;1];
    end
    %%%%%Velocity alignment term%%%%%
    vijFrictMax = max(v_frict,Dfunction(dis_to_neighbor - r_frict_0,a_frict,p_frict));
    velijInCom = repmat(vel2DId,1,length(dis_to_neighbor)) - vel2D_neighbor;
    vijInCom = sqrt(sum(velijInCom.^2,1));
    
    inFrictInCom = find(vijInCom > vijFrictMax);
    vFrictId = zeros(dim,1);
    if ~isempty(inFrictInCom)
        vijInFrict = repmat(vijInCom(inFrictInCom),dim,1);
        vijFrictMaxInFrict = repmat(vijFrictMax(inFrictInCom),dim,1);
        vFrictId = -c_frict * sum((vijInFrict - vijFrictMaxInFrict).*velijInCom(:,inFrictInCom)./vijInFrict,2);
    end
end
%%%%%Avoidance obstacles term%%%%%
vShillId = zeros(dim,1);

posisArray = repmat(pos2DId,1,size(pos_shill,2)) - pos_shill; disis = sqrt(sum(posisArray.^2,1));
inComS = find(disis < r_com); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disisInCom = disis(inComS);

visFrictMax = Dfunction(disisInCom - r_shill_0,a_shill,p_shill);
velisInCom = repmat(vel2DId,1,length(disisInCom)) - v_shill * vel_shill(:,inComS);
visInCom = sqrt(sum(velisInCom.^2,1));

inFrictInComS = find(visInCom > visFrictMax);

if ~isempty(inFrictInComS)
    visInFrict = repmat(visInCom(inFrictInComS),dim,1);
    visFrictMaxInFrict = repmat(visFrictMax(inFrictInComS),dim,1);
    vShillId = - sum((visInFrict - visFrictMaxInFrict).*velisInCom(:,inFrictInComS)./visInFrict,2);
end
%%%%%Final equation of desired velocity%%%%%
velDesired_id_2D = vFlockId + vRepId + vFrictId + vShillId;

% Clamp
vel_norm = norm(velDesired_id_2D);
if vel_norm > v_max
    velDesired_id_2D = velDesired_id_2D./vel_norm* v_max;
end
velDesired_id(1:2) = velDesired_id_2D;

end

