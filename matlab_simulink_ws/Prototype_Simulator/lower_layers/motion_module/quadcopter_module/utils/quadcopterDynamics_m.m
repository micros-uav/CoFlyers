function sdot = quadcopterDynamics_m(t, s, w, params, h_ground)
%QUADCOPTERDYNAMICS Solve quadcoptor equation of motion
%   readonly
%   calculate the derivative of the state vector

num = size(s,2);
% num = floor(length(s)/13); % Number of quadcopters
% s = reshape(s,13,num);     % States
% w = reshape(w,4,num);      % rotation velocity of propellers

mass       = params(1); 
Ixx        = params(2);
Iyy        = params(3);
Izz        = params(4);
arm_length = params(5);
ct         = params(6);
cm         = params(7);
I = diag([Ixx,Iyy,Izz]);
invI = diag([1/Ixx,1/Iyy,1/Izz]);

dx = arm_length*0.7071;
ct_dx = ct*dx;
gravity = 9.81;

S = [   ct,    ct,    ct,    ct
    -ct_dx, ct_dx, ct_dx,-ct_dx
    -ct_dx,-ct_dx, ct_dx, ct_dx
       -cm,    cm,   -cm,    cm];

T_tau = S * w.^2;

F = T_tau(1,:);
M = T_tau(2:4,:);

% Assign states
% x = s(1,:);
% y = s(2,:);
z = s(3,:);
xdot = s(4,:);
ydot = s(5,:);
zdot = s(6,:);
% qW = s(7,:);
% qX = s(8,:);
% qY = s(9,:);
% qZ = s(10,:);
p = s(11,:);
q = s(12,:);
r = s(13,:);

quat = s(7:10,:); % [qW; qX; qY; qZ]
quatNorm2 = sum(quat.^2,1);
quat = quat./repmat(sqrt(quatNorm2),4,1);

b2w = quaternion2RotationMatrix(quat);

% Acceleration
accel = 1 / mass * reshape(b2w(:,3,:),3,num).*repmat(F,3,1);
temp = z>h_ground;
accel(3,temp) = accel(3,temp) - repmat(gravity,1,sum(temp));
% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = ones(1,num) - quatNorm2;
% qdot = -1/2*[0, -p, -q, -r;...
%              p,  0, -r,  q;...
%              q,  r,  0, -p;...
%              r, -q,  p,  0] * quat + K_quat*repmat(quaterror,4,1) .* quat;
qdot = zeros(4,num);
qdot(1,:) = 1/2*sum([zeros(1,num);-p;-q;-r].*quat,1) + K_quat*quaterror.*quat(1,:);
qdot(2,:) = 1/2*sum([p;zeros(1,num);r;-q].*quat,1) + K_quat*quaterror.*quat(2,:);
qdot(3,:) = 1/2*sum([q;-r;zeros(1,num);p].*quat,1) + K_quat*quaterror.*quat(3,:);
qdot(4,:) = 1/2*sum([r;q;-p;zeros(1,num)].*quat,1) + K_quat*quaterror.*quat(4,:);
% Angular acceleration
omega = [p;q;r];
% pqrdot   = invI * (M - cross(omega, I*omega));
pqrdot   = invI * (M - my_cross(omega, I*omega));
% Assemble sdot
sdot = zeros(13,num);
sdot(1,:)  = xdot;
sdot(2,:)  = ydot;
sdot(3,:)  = zdot;
sdot(4:6,:)  = accel;
% sdot(5,:)  = accel(2,:);
% sdot(6,:)  = accel(3,:);
sdot(7:10,:)  = qdot(1:4,:);
% sdot(8,:)  = qdot(2,:);
% sdot(9,:)  = qdot(3,:);
% sdot(10,:) = qdot(4,:);
sdot(11:13,:) = pqrdot;
% sdot(12,:) = pqrdot(2,:);
% sdot(13,:) = pqrdot(3,:);
% sdot = sdot(:);
end

