function [r_com,...
v_flock,...
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
vel_shill] = Vasarhelyi_module_parameters_5()
%VASARHELYI_MODULE_PARAMETERS_5 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
r_com = 4.000000000000;
v_flock = 0.200000000000;
r_rep_0 = 1.183100000000;
p_rep = 0.507130000000;
r_frict_0 = 6.084000000000;
c_frict = 0.205120000000;
v_frict = 0.021842000000;
p_frict = 5.229600000000;
a_frict = 0.568140000000;
r_shill_0 = 0.673320000000;
v_shill = 0.578010000000;
p_shill = 4.443300000000;
a_shill = 0.531250000000;
v_max = 0.240000000000;
dim = 2.000000000000;
height = 0.700000000000;
dr_shill = 0.100000000000;
pos_shill = [-4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, -1.100000000000, -1.000000000000, -0.900000000000, -0.800000000000, -0.700000000000, -0.600000000000, -0.500000000000, -0.400000000000, -0.300000000000, -0.200000000000, -0.100000000000, 0.000000000000, 0.100000000000, 0.200000000000, 0.300000000000, 0.400000000000, 0.500000000000, 0.600000000000, 0.700000000000, 0.800000000000, 0.900000000000, 1.000000000000, 1.100000000000
-4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, -4.000000000000, -3.900000000000, -3.800000000000, -3.700000000000, -3.600000000000, -3.500000000000, -3.400000000000, -3.300000000000, -3.200000000000, -3.100000000000, -3.000000000000, -2.900000000000, -2.800000000000, -2.700000000000, -2.600000000000, -2.500000000000, -2.400000000000, -2.300000000000, -2.200000000000, -2.100000000000, -2.000000000000, -1.900000000000, -1.800000000000, -1.700000000000, -1.600000000000, -1.500000000000, -1.400000000000, -1.300000000000, -1.200000000000, 4.000000000000, 3.900000000000, 3.800000000000, 3.700000000000, 3.600000000000, 3.500000000000, 3.400000000000, 3.300000000000, 3.200000000000, 3.100000000000, 3.000000000000, 2.900000000000, 2.800000000000, 2.700000000000, 2.600000000000, 2.500000000000, 2.400000000000, 2.300000000000, 2.200000000000, 2.100000000000, 2.000000000000, 1.900000000000, 1.800000000000, 1.700000000000, 1.600000000000, 1.500000000000, 1.400000000000, 1.300000000000, 1.200000000000, 1.100000000000, 1.000000000000, 0.900000000000, 0.800000000000, 0.700000000000, 0.600000000000, 0.500000000000, 0.400000000000, 0.300000000000, 0.200000000000, 0.100000000000, 0.000000000000, -0.100000000000, -0.200000000000, -0.300000000000, -0.400000000000, -0.500000000000, -0.600000000000, -0.700000000000, -0.800000000000, -0.900000000000, -1.000000000000, -1.100000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, -4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000, 4.000000000000];
vel_shill = [1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000
-0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000];


end