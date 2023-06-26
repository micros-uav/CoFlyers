function [posDesired_id,velDesired_id,accDesired_id,control_mode_id] = Vasarhelyi_module_generate_desire_i(id,state_i,states_neighbor,...
        dis_to_neighbor,posid_to_neighbor)
%VASARHELYI_MODULE_GENERATE_DESIRE_I Summary of this function goes here
%   Detailed explanation goes here

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
vel_shill] = Vasarhelyi_module_parameters();
VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;

posDesired_id = [state_i(1:2);height;0];
velDesired_id = zeros(4,1);
accDesired_id = zeros(4,1);
control_mode_id = VELOCITY_HORIZONTAL_CONTROL_TYPE;

pos2DId = state_i(1:2);
vel2DId = state_i(4:5);
vel2D_neighbor = states_neighbor(4:5,:);
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

