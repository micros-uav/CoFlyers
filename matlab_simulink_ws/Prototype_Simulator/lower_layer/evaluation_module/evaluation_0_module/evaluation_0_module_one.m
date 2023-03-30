function [values, values_for_visual] = evaluation_0_module_one(states,parameters_evalue,map_grid,parameters_map)
%EVALUATION_MODEL_ONE 

 [v_flock,rColl,~,~,~] =...
    evaluation_0_module_parameters_deal(parameters_evalue);

number = size(states,2); 
position = states(1:2,:);
velocity = states(4:5,:);

if number > 1
    disMat = pdist(position');
    phiColl = mean(disMat < rColl);  %#1
%     temp = squareform(disMat);
%     temp(1:number+1:number*number) = inf;
%     phiNND = mean(min(temp,[],1))/rColl;
    phiMND = rColl/min(disMat);
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
outOfMap = map_module_out_of_map(states(1,:),states(2,:),parameters_map,map_grid);
phiWall = sum(outOfMap)/number;
values = [phiCorr;phiVel;phiColl;phiWall;phiMND];

%%% For visual
values_for_visual = atan2(velUnit(2,:),velUnit(1,:)); % Angles
end