function [posDesired_id,velDesired_id,accDesired_id,control_mode_id] = Vasarhelyi_module_generate_desire_i(id,state_i,states_neighbor,...
        dis_to_neighbor,posid_to_neighbor,parameters_flocking,map_lines,map_grid,datas_sensor_env)
%VASARHELYI_MODULE_GENERATE_DESIRE_I Summary of this function goes here
%   Detailed explanation goes here
[r_com,v_flock,r_rep0,p_rep,r_frict0,C_frict,v_frict,p_frict,a_frict,r_shill0,...
    v_shill,p_shill,a_shill,v_max,~,heights,~,posShill,velShill] = Vasarhelyi_module_parameters_deal(parameters_flocking);
dimension = 2;

VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;

posDesired_id = [state_i(1:2);heights(id);0];
velDesired_id = zeros(4,1);
accDesired_id = zeros(4,1);
control_mode_id = VELOCITY_HORIZONTAL_CONTROL_TYPE;

pos2DId = state_i(1:2);
vel2DId = state_i(4:5);
vel2D_neighbor = states_neighbor(4:5,:);
%Self-propelling term
velIdNorm = norm(vel2DId);
if velIdNorm == 0
    velRandom = rand(dimension,1); velRandom = velRandom/norm(velRandom);
    vFlockId = v_flock * velRandom;
else
    vFlockId = v_flock * vel2DId/velIdNorm;
end
%    vFlockId = v_flock * [1;0];

vRepId = zeros(2,1);
vFrictId = zeros(2,1);
if ~isempty(dis_to_neighbor)
    %%%%%Repulsion term%%%%%
    inRepInCom = find(dis_to_neighbor<r_rep0);
    vRepId = zeros(dimension,1);
    if ~isempty(inRepInCom)
        disijInRep = repmat(dis_to_neighbor(inRepInCom),dimension,1);
        vRepId = p_rep * sum((r_rep0 - disijInRep) .* posid_to_neighbor(:,inRepInCom)./disijInRep,2);
%         vRepId = [1;1];
    end
    %%%%%Velocity alignment term%%%%%
    vijFrictMax = max(v_frict,Dfunction(dis_to_neighbor - r_frict0,a_frict,p_frict));
    velijInCom = repmat(vel2DId,1,length(dis_to_neighbor)) - vel2D_neighbor;
    vijInCom = sqrt(sum(velijInCom.^2,1));
    
    inFrictInCom = find(vijInCom > vijFrictMax);
    vFrictId = zeros(dimension,1);
    if ~isempty(inFrictInCom)
        vijInFrict = repmat(vijInCom(inFrictInCom),dimension,1);
        vijFrictMaxInFrict = repmat(vijFrictMax(inFrictInCom),dimension,1);
        vFrictId = -C_frict * sum((vijInFrict - vijFrictMaxInFrict).*velijInCom(:,inFrictInCom)./vijInFrict,2);
    end
end
%%%%%Avoidance obstacles term%%%%%
vShillId = zeros(dimension,1);

posisArray = repmat(pos2DId,1,size(posShill,2)) - posShill; disis = sqrt(sum(posisArray.^2,1));
inComS = find(disis < r_com); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disisInCom = disis(inComS);

visFrictMax = Dfunction(disisInCom - r_shill0,a_shill,p_shill);
velisInCom = repmat(vel2DId,1,length(disisInCom)) - v_shill * velShill(:,inComS);
visInCom = sqrt(sum(velisInCom.^2,1));

inFrictInComS = find(visInCom > visFrictMax);

if ~isempty(inFrictInComS)
    visInFrict = repmat(visInCom(inFrictInComS),dimension,1);
    visFrictMaxInFrict = repmat(visFrictMax(inFrictInComS),dimension,1);
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

