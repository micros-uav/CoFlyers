function RotationMatrix = euler2RotationMatrix(euler)
%EULER2ROTATIONMATRIX 此处显示有关此函数的摘要
%   ZXY: Yaw, Roll, Pitch
%   body vector to world vector
num = size(euler,2);
yaw = reshape(euler(1,:),1,1,num);
roll = reshape(euler(2,:),1,1,num);
pitch = reshape(euler(3,:),1,1,num);

c1 = cos(yaw); s1 = sin(yaw);
c2 = cos(roll); s2 = sin(roll);
c3 = cos(pitch); s3 = sin(pitch);

RotationMatrix = [c1.*c3 - s1.*s2.*s3, -c2.*s1, c1.*s3 + c3.*s1.*s2
                  c3.*s1 + c1.*s2.*s3,  c1.*c2, s1.*s3 - c1.*c3.*s2
                              -c2.*s3,      s2,              c2.*c3];
end

