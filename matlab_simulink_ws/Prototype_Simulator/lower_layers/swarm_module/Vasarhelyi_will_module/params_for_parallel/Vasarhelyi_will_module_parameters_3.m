function [r_com,...
v_flock,...
tau_will,...
r_rep_0,...
p_rep,...
r_frict_0,...
c_frict,...
v_frict,...
p_frict,...
a_frict,...
r_shill_0,...
v_shill,...
p_shill,...
a_shill,...
v_max,...
dim,...
height,...
dr_shill,...
pos_shill,...
vel_shill] = Vasarhelyi_will_module_parameters_3()
%VASARHELYI_WILL_MODULE_PARAMETERS_3 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
r_com = 4.000000000000;
v_flock = 0.200000000000;
tau_will = 1.000000000000;
r_rep_0 = 0.861723000000;
p_rep = 0.923553000000;
r_frict_0 = 5.693489000000;
c_frict = 0.155726000000;
v_frict = 0.057981000000;
p_frict = 4.506540000000;
a_frict = 0.401540000000;
r_shill_0 = 0.968532000000;
v_shill = 0.943399000000;
p_shill = 1.980570000000;
a_shill = 0.223193000000;
v_max = 0.240000000000;
dim = 2.000000000000;
height = 0.700000000000;
dr_shill = 0.100000000000;
pos_shill = [-4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, -1.100000000000, -1.000000000000, -0.900000000000, -0.800000000000, -0.700000000000, -0.600000000000, -0.500000000000, -0.400000000000, -0.300000000000, -0.200000000000, -0.100000000000, 0.000000000000, 0.100000000000, 0.200000000000, 0.300000000000, 0.400000000000, 0.500000000000, 0.600000000000, 0.700000000000, 0.800000000000, 0.900000000000, 1.000000000000, 1.100000000000
-4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000];
vel_shill = [1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000
-0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000];


end