function [r_com,v_flock,tau_will,r_rep0,p_rep,r_frict0,C_frict,v_frict,p_frict,a_frict,r_shill0,...
    v_shill,p_shill,a_shill,v_max,number_h,heights,numS,posShill,velShill] =...
    Vasarhelyi_will_module_parameters_deal(parameters_flocking)
%VASARHELYI_WILL_MODULE_PARAMETERS_DEAL Summary of this function goes here
%   Detailed explanation goes here
myCount = 1;
r_com    = parameters_flocking(myCount); myCount = myCount + 1;
v_flock  = parameters_flocking(myCount); myCount = myCount + 1;
tau_will  = parameters_flocking(myCount); myCount = myCount + 1;
r_rep0   = parameters_flocking(myCount); myCount = myCount + 1;
p_rep    = parameters_flocking(myCount); myCount = myCount + 1;
r_frict0 = parameters_flocking(myCount); myCount = myCount + 1;
C_frict  = parameters_flocking(myCount); myCount = myCount + 1;
v_frict  = parameters_flocking(myCount); myCount = myCount + 1;
p_frict  = parameters_flocking(myCount); myCount = myCount + 1;
a_frict  = parameters_flocking(myCount); myCount = myCount + 1;
r_shill0 = parameters_flocking(myCount); myCount = myCount + 1;
v_shill  = parameters_flocking(myCount); myCount = myCount + 1;
p_shill  = parameters_flocking(myCount); myCount = myCount + 1;
a_shill  = parameters_flocking(myCount); myCount = myCount + 1;
v_max    = parameters_flocking(myCount); myCount = myCount + 1;
number_h   = parameters_flocking(myCount); myCount = myCount + 1;
heights  = parameters_flocking(myCount:myCount+number_h-1); myCount = myCount + number_h;
numS     = parameters_flocking(myCount); myCount = myCount + 1;
dCount = (numS-1)*2 + 1;
posShill  = reshape(parameters_flocking(myCount:myCount + dCount),2,numS); myCount = myCount + dCount + 1;
velShill  = reshape(parameters_flocking(myCount:myCount + dCount),2,numS); myCount = myCount + dCount + 1;

end

