function w_d = attitudeController_m(YrRPT_d,YRP_pqr,params)
%ATTITUDECONTROLLER_M 此处显示有关此函数的摘要
%   此处显示详细说明
Ixx = params(1);
Iyy = params(2);
Izz = params(3);
kpx = params(4)/Ixx;
kpy = params(5)/Iyy;
kpz = params(6)/Izz;
kdx = params(7)/Ixx;
kdy = params(8)/Iyy;
kdz = params(9)/Izz;
lenArm = params(10);
ct  = params(11);
cm  = params(12);

dx = lenArm*0.7071;
dxct = dx*ct;

num = size(YrRPT_d,2);

M_d = zeros(3,num);
M_d(1,:) = (kpx * (YrRPT_d(2,:) - YRP_pqr(2,:)) + kdx * (0 - YRP_pqr(4,:))) * Ixx;
M_d(2,:) = (kpy * (YrRPT_d(3,:) - YRP_pqr(3,:)) + kdy * (0 - YRP_pqr(5,:))) * Iyy;
M_d(3,:) = (kdz * (YrRPT_d(1,:) - YRP_pqr(6,:))) * Izz;

thrust_d = YrRPT_d(4,:);

Sinv = [1/ct,-1/dxct,-1/dxct,-1/cm
        1/ct, 1/dxct,-1/dxct, 1/cm
        1/ct, 1/dxct, 1/dxct,-1/cm
        1/ct,-1/dxct, 1/dxct, 1/cm]/4;
w_d = Sinv*[thrust_d;M_d];
w_d(w_d<0) = 0;
w_d = sqrt(w_d);
end

