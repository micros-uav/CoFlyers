function plotMultiQuadcopters_pm(myFigure,XY,height,lenArm)
%PLOTMULTIQUADCOPTERS 此处显示有关此函数的摘要
%   此处显示详细说明

number = size(XY,2);
xyzs = [XY;height*ones(1,number)];
% eulers = zeros(3,number);

% rotationMatrixs = euler2RotationMatrix(eulers([2,1,3],:)','ZXY');
% rotationMatrixs = euler2RotationMatrix(eulers);

points4 = lenArm*0.7071*[[1;1;0],[1;-1;0],[-1;-1;0],[-1;1;0]];
points4s = repmat(points4,1,1,number);
% points4s = pagemtimes(rotationMatrixs,points4s);
% points4x = points4(1,:);
% points4y = points4(2,:);
% points4z = points4(3,:);
points4xs = reshape(points4s(1,:,:),4,number);
points4ys = reshape(points4s(2,:,:),4,number);
points4zs = reshape(points4s(3,:,:),4,number);

points4xs = points4xs + repmat(xyzs(1,:),4,1);
points4ys = points4ys + repmat(xyzs(2,:),4,1);
points4zs = points4zs + repmat(xyzs(3,:),4,1);

% points4s = repmat(points4,1,number,1) + repmat(xyzs,1,1,4);

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

