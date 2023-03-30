function plotMultiQuadcopters(myFigure,states,lenArm)
%PLOTMULTIQUADCOPTERS 此处显示有关此函数的摘要
%   此处显示详细说明

number = size(states,2);
xyzs = states(1:3,:);

points4 = lenArm*0.7071*[[1;1;0],[1;-1;0],[-1;-1;0],[-1;1;0]];
points4s = repmat(points4,1,1,number);

% Rotation, but pagemtimes is only supportted by more than 2020a version.
% eulers = states(7:9,:);
% rotationMatrixs = euler2RotationMatrix(eulers);
% points4s = pagemtimes(rotationMatrixs,points4s);

points4xs = reshape(points4s(1,:,:),4,number);
points4ys = reshape(points4s(2,:,:),4,number);
points4zs = reshape(points4s(3,:,:),4,number);

points4xs = points4xs + repmat(xyzs(1,:),4,1);
points4ys = points4ys + repmat(xyzs(2,:),4,1);
points4zs = points4zs + repmat(xyzs(3,:),4,1);

myFigure;
% hold on
% grid on
% box on
% axis equal
% mycolors = lines(number);
plot3(points4xs,points4ys,points4zs,'.','MarkerSize',10,'Color','r')

plot3(points4xs([1,3],:),points4ys([1,3],:),points4zs([1,3],:),'-','LineWidth',2,'Color','k')
plot3(points4xs([2,4],:),points4ys([2,4],:),points4zs([2,4],:),'-','LineWidth',2,'Color','k')
end

