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
vel_shill] = Vasarhelyi_module_parameters_1()
%VASARHELYI_MODULE_PARAMETERS_1 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
r_com = 8.000000000000;
v_flock = 0.200000000000;
r_rep_0 = 1.881200000000;
p_rep = 0.435910000000;
r_frict_0 = 4.292200000000;
c_frict = 0.371930000000;
v_frict = 0.123610000000;
p_frict = 2.726700000000;
a_frict = 0.975050000000;
r_shill_0 = 0.697860000000;
v_shill = 0.906140000000;
p_shill = 4.654900000000;
a_shill = 0.507180000000;
v_max = 0.240000000000;
dim = 2.000000000000;
height = 0.700000000000;
dr_shill = 1.000000000000;
pos_shill = [-20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, -20.000000000000, -19.000000000000, -18.000000000000, -17.000000000000, -16.000000000000, -15.000000000000, -14.000000000000, -13.000000000000, -12.000000000000, -11.000000000000, -10.000000000000, -9.000000000000, -8.000000000000, -7.000000000000, -6.000000000000, 20.000000000000, 19.000000000000, 18.000000000000, 17.000000000000, 16.000000000000, 15.000000000000, 14.000000000000, 13.000000000000, 12.000000000000, 11.000000000000, 10.000000000000, 9.000000000000, 8.000000000000, 7.000000000000, 6.000000000000, 5.000000000000, 4.000000000000, 3.000000000000, 2.000000000000, 1.000000000000, 0.000000000000, -1.000000000000, -2.000000000000, -3.000000000000, -4.000000000000, -5.000000000000, 20.000000000000, 19.000000000000, 18.000000000000, 17.000000000000, 16.000000000000, 15.000000000000, 14.000000000000, 13.000000000000, 12.000000000000, 11.000000000000, 10.000000000000, 9.000000000000, 8.000000000000, 7.000000000000, 6.000000000000, -20.000000000000, -19.000000000000, -18.000000000000, -17.000000000000, -16.000000000000, -15.000000000000, -14.000000000000, -13.000000000000, -12.000000000000, -11.000000000000, -10.000000000000, -9.000000000000, -8.000000000000, -7.000000000000, -6.000000000000, -5.000000000000, -4.000000000000, -3.000000000000, -2.000000000000, -1.000000000000, 0.000000000000, 1.000000000000, 2.000000000000, 3.000000000000, 4.000000000000, 5.000000000000
-20.000000000000, -19.000000000000, -18.000000000000, -17.000000000000, -16.000000000000, -15.000000000000, -14.000000000000, -13.000000000000, -12.000000000000, -11.000000000000, -10.000000000000, -9.000000000000, -8.000000000000, -7.000000000000, -6.000000000000, 20.000000000000, 19.000000000000, 18.000000000000, 17.000000000000, 16.000000000000, 15.000000000000, 14.000000000000, 13.000000000000, 12.000000000000, 11.000000000000, 10.000000000000, 9.000000000000, 8.000000000000, 7.000000000000, 6.000000000000, 5.000000000000, 4.000000000000, 3.000000000000, 2.000000000000, 1.000000000000, 0.000000000000, -1.000000000000, -2.000000000000, -3.000000000000, -4.000000000000, -5.000000000000, -20.000000000000, -19.000000000000, -18.000000000000, -17.000000000000, -16.000000000000, -15.000000000000, -14.000000000000, -13.000000000000, -12.000000000000, -11.000000000000, -10.000000000000, -9.000000000000, -8.000000000000, -7.000000000000, -6.000000000000, 20.000000000000, 19.000000000000, 18.000000000000, 17.000000000000, 16.000000000000, 15.000000000000, 14.000000000000, 13.000000000000, 12.000000000000, 11.000000000000, 10.000000000000, 9.000000000000, 8.000000000000, 7.000000000000, 6.000000000000, 5.000000000000, 4.000000000000, 3.000000000000, 2.000000000000, 1.000000000000, 0.000000000000, -1.000000000000, -2.000000000000, -3.000000000000, -4.000000000000, -5.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, -20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000, 20.000000000000];
vel_shill = [1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000
-0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, -0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, 1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000, -1.000000000000];


end