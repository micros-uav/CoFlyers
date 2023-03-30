function RotationMatrix = quaternion2RotationMatrix(quaternion)
%quaternion2RotationMatrix Converts a Quaternion to Rotation matrix
%   support the calculation of multiple quaternion 
%   written by Jialei Huang

num = size(quaternion,2);
% normalize q
quaternion = quaternion./sqrt(sum(quaternion.^2,1));

qahat = zeros(3,3,num);
qahat(1,2,:) = -quaternion(4,:);
qahat(1,3,:) = quaternion(3,:);
qahat(2,3,:) = -quaternion(2,:);
qahat(2,1,:) = quaternion(4,:);
qahat(3,1,:) = -quaternion(3,:);
qahat(3,2,:) = quaternion(2,:);

RotationMatrix = repmat(eye(3),1,1,num) + 2*pagemtimes(qahat,qahat) + 2*repmat(reshape(quaternion(1,:),1,1,num),3,3,1).*qahat;
end

