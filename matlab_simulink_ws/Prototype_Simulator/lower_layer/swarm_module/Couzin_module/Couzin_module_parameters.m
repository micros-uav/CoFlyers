function parameters_flocking = Couzin_module_parameters(number_h)
%COUZIN_MODULE_PARAMETERS Summary of this function goes here
%   Detailed explanation goes here

v_flock = 3.0;            % move speed
% Swarm
% r_rep = 1;  
% dro   = 0;  
% dra    = 14;  
% Torus
r_rep = 1;                  % radius of repulsion zone
dro   = 2;                  % radius differnce of alignment zone
dra    = 14;               % radius differnce of attraction zone
% Dynamic Parallel
% r_rep = 1;                  % radius of repulsion zone
% dro   = 3;                  % radius differnce of alignment zone
% dra    = 14;               % radius differnce of attraction zone
% Parallel
% r_rep = 1;                  % radius of repulsion zone
% dro   = 10;                  % radius differnce of alignment zone
% dra    = 14;               % radius differnce of attraction zone


alpha = 135/180*pi; % half field of view 
sigma = 5/180*pi;     % angle noise

% r_ori = r_rep + heaviside(dro)*dro;
% r_att = r_ori + heaviside(dra)*dra;

dim = 3;
if dim ==2
    num = number_h;
    temp = magic(ceil(sqrt(number_h)));
    heights = temp(1:number_h)/max(temp,[],'all')*0.2+0.5;
else
    num = 0;
    heights = [];
end

%%% Merge all parameters to an array
parameters_flocking = [
    alpha
    dim;
    v_flock;
    r_rep
    dro
    dra;
    sigma
    num
    heights(:)];

end