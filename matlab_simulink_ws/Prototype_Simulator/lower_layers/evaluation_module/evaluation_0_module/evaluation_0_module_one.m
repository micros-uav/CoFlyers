function values = evaluation_0_module_one(t, states, map3d_faces, map3d_struct)
%EVALUATION_MODEL_ONE 

file_name_param = 'evaluation_0_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[v_flock,...
r_coll,...
a_tol] = fun_params();


number = size(states,2); 
position = states(1:2,:);
velocity = states(4:5,:);

if number > 1
    disMat = pdist(position');
    phiColl = mean(disMat < r_coll);  %#1
%     temp = squareform(disMat);
%     temp(1:number+1:number*number) = inf;
%     phiNND = mean(min(temp,[],1))/rColl;
    phiMND = r_coll/min(disMat);
else
    phiColl = 0;
    phiMND = 0;
end
velocity = velocity + rand(size(velocity))*1e-20;
speed = vecnorm(velocity,2,1);
velUnit = velocity./speed;
if number > 1
    phiCorr = (norm(mean(velUnit,2))^2*number - 1)/(number - 1); %#2
else
    phiCorr = norm(mean(velUnit,2))^2;
end
phiVel = mean(speed)/v_flock;                                    %#3

% outOfMap = find(position(1,:) < (min(map(1,:)) + rColl) |...
%                 position(1,:) > (max(map(1,:)) - rColl) | ...
%                 position(2,:) < (min(map(2,:)) + rColl) | ...
%                 position(2,:) > (max(map(2,:)) - rColl));
% outOfMap = map_module_out_of_map(states(1,:),states(2,:),parameters_map,map_grid);

out_of_map = map_module_out_of_map(states(1:3,:),map3d_struct);
col_map = map_module_collision_detection(states(1:3,:),map3d_faces,r_coll/2);
outOfMap = out_of_map | col_map;

phiWall = sum(outOfMap)/number;
values = [phiCorr;phiVel;phiColl;phiWall;phiMND];

end