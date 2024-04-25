function [range_s,psi_s,phi_s] = get_lidar_from_elevation_map(pos_agent, att_agent, terrain, terrain_params, resolution, r_min,  r_sense, phi_range, psi_range)
%GET_LIDAR_FROM_MAP3D_FACES
% att_agent: degree
% The current feature does not support the altitude angle direction.

% Initialization of the discrete angles
n_dangle = max(ceil((psi_range(2)-psi_range(1))/resolution), 1);
psi_s = linspace(psi_range(1),psi_range(2),n_dangle);
phi_s = zeros(size(psi_s)); %%%%% Now no support
range_s = zeros(1,size(psi_s,2)) + inf;

% No use terrain
if isempty(terrain)
    return;
end
% 
[h,w] = size(terrain);
t_p = terrain_params(:,1); % Translation properties of the elevation map
t_s = terrain_params(:,2); % Scaling properties of the elevation map
r_s_x = floor(r_sense/t_s(1)); % Sense range, pixel
r_s_y = floor(r_sense/t_s(2)); %
p_x = floor((pos_agent(1) - t_p(1))/t_s(1)); % Pixel coordinates of the agent
p_y = floor((pos_agent(2) - t_p(2))/t_s(2));

if terrain(p_y,p_x)>=pos_agent(3)
    % The agent is inside the terrain.
    range_s = range_s*0;
else
    % Get local elevation map within distance 'r_sense'.
    ind_x = p_x-r_s_x:p_x+r_s_x;
    ind_y = p_y-r_s_y:p_y+r_s_y;
    ind_x(ind_x<1|ind_x>w) = [];
    ind_y(ind_y<1|ind_y>h) = [];
    map_local = terrain(ind_y,ind_x); % Elevation map within perceptual range
    % Find obstacles in the same height
    [sub_y_obs, sub_x_obs] = find(map_local>pos_agent(3)); % Any point higher than the height of the agent is considered an obstacle point
    if ~isempty(sub_y_obs)
        pos_x_obs = (sub_x_obs - r_s_x - 1) *t_s(1); % To real coordinates, relative
        pos_y_obs = (sub_y_obs - r_s_y - 1) *t_s(2);
        w_obs = 1*t_s(1);
        h_obs = 1*t_s(2);
        angle_obs = atan2(pos_y_obs, pos_x_obs);
        % delta_dis = h_obs/2./sin(abs(angle_obs));
        % delta_dis(isinf(delta_dis)) = w_obs/2;
        delta_dis = w_obs/2;
        dis_obs_0 = sqrt(pos_y_obs.^2 +  pos_x_obs.^2);
        dis_obs =  max( dis_obs_0 - delta_dis, 0);
        r_obs = h_obs;
        % r_obs = sqrt(w_obs^2+h_obs^2);
        dangle_obs_half = asin(r_obs./dis_obs_0);
        % n_temp = ceil(dangle_obs_half./resolution);
        n_max = ceil(size(psi_s,2)/4/2);
        temp1 = repmat(-n_max:n_max,size(dis_obs,1),1);
        temp2 = dangle_obs_half/n_max;
        temp1 = wrapToPi(temp1.*temp2 + angle_obs);
        angle_temp = temp1(:);
        dis_temp = repmat(dis_obs,2*n_max+1,1);
        out_of_angle = find(angle_temp<psi_range(1)|angle_temp>psi_range(2));
        angle_temp(out_of_angle) = [];
        dis_temp(out_of_angle) = [];
        ind_temp = ceil((angle_temp-psi_range(1))/resolution);
        A = sortrows([ind_temp,dis_temp]);
        [ind_new, ia] = unique(A(:,1));
        dis_new = A(ia,2);

        range_s(ind_new) = dis_new;
        t3 = att_agent(3);
        ind3 = ceil(t3/resolution);
        range_s = circshift(range_s,-ind3);
        range_s(range_s>r_sense|range_s<r_min) = inf;
    end
end
end

