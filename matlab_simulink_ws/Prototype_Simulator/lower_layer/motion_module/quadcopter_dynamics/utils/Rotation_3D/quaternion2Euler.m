function euler = quaternion2Euler(quaternion)
%QUATERNION2EULER 此处显示有关此函数的摘要
%   此处显示详细说明
num = size(quaternion,2);
euler = zeros(3,num);
% normalize q
quaternion = quaternion./sqrt(sum(quaternion.^2,1));

qw = quaternion(1,:);
qx = quaternion(2,:);
qy = quaternion(3,:);
qz = quaternion(4,:);

% ZXY: Yaw, Roll, Pitch
euler(1,:) = atan2(-2*qx.*qy + 2*qw.*qz,- 2*qx.^2 - 2*qz.^2 + 1);
euler(2,:) = asin(2*qw.*qx + 2*qy.*qz);
euler(3,:) = atan2(-2*qx.*qz + 2*qw.*qy,- 2*qx.^2 - 2*qy.^2 + 1);

end

