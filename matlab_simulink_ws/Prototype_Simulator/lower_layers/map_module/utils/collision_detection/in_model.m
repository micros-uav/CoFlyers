function [in_s, d_min_s] = in_model(point3d_s, face_s_model, resolution)
%IN_MODEL Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    resolution = pi/5;
end

num_p = size(point3d_s,2);

in_s = false(1,num_p);
d_min_s = zeros(1,num_p) + inf;

r_sense = 100000;
phi_range = [-pi/2,pi/2];
psi_range = [-pi,pi];

for i = 1:num_p
    [range_s,psi_s,phi_s] = get_lidar_from_map3d(point3d_s(:,i), [0;0;0], face_s_model, resolution, 0, r_sense, phi_range, psi_range);

    ind = find(range_s==inf);
    per = length(ind)/numel(range_s);
    if per < 0.01
        in_s(i) = true;
    end
    d_min_s(i) = min(d_min_s(i),min(range_s,[],'all'));
end

end

