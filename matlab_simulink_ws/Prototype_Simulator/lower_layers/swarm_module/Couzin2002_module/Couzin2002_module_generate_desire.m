function [command_upper_s,control_mode_s] =Couzin2002_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params)
%COUZIN2002_MODULE_GENERATE_DESIRE Generate the desired position and velocity
% Automatically generated once by read_parameter_xml.m
% This function will be called by swarms_module_generate_desire.m
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];
% control_mode:
% TAKEOFF_TYPE = 2;
% HOVER_TYPE = 3;
% LAND_TYPE = 4;
% POSITION_CONTROL_TYPE = 5;
% VELOCITY_CONTROL_TYPE = 6;
% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;


% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your swarm submodule, you need to
% get parameters by Couzin2002_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'Couzin2002_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[v_flock,...
r_rep,...
dro,...
dra,...
alpha,...
sigma,...
dim,...
height] = fun_params();

sigma = sigma*180/pi;

r_com = r_rep+dro+dra;

%
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);

if dim == 3
    control_mode_s = uint8(zeros(1,number))+6;
else
    control_mode_s = uint8(zeros(1,number))+7;
end


position_s = states(1:3,:);
velocity_s = states(4:6,:);
velocity_s_unit = velocity_s./(vecnorm(velocity_s)+1e-8);

v_d = Collective_behavior_Couzin2002(position_s,velocity_s_unit,number,r_rep,r_rep+dro,r_rep+dro+dra,alpha*2*180/pi);

v_d = add_noise_to_vel(v_d,sigma);

command_upper_s(5:7,:) = v_flock*v_d;



    function d = Collective_behavior_Couzin2002(c,V,N,rr,ro,ra,alpha)
        %Collective_behavior
        %==========================================================================
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rij_x = c(1,:)-c(1,:)';
        rij_y = c(2,:)-c(2,:)';
        rij_z = c(3,:)-c(3,:)';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        r_matrix = sqrt(rij_x.^2 + rij_y.^2 + rij_z.^2);

        % alpha_ij = acosd(dot(Vi,rij)/|Vi|/|rij|)
        V_rij_dot = (V(1,:)'.*rij_x) +  (V(2,:)'.*rij_y) +  (V(3,:)'.*rij_z);
        V_norm = sqrt(sum(V.^2));
        temp = my_saturation(V_rij_dot./V_norm'./r_matrix,-1,1);
        alpha_matrix = acosd(temp);
        alpha_matrix(1:N+1:N*N) = 0;
        in_view = alpha_matrix < alpha/2;
        %==========================================================================

        %==========================================================================
        zone_rr = r_matrix<rr & in_view;
        zone_rr(1:N+1:N*N) = 0;
        zone_ro = r_matrix >=rr & r_matrix <ro  & in_view;
        zone_ra = r_matrix >=ro & r_matrix <ra  & in_view;
        nr = sum(zone_rr,2);
        no = sum(zone_ro,2);
        na = sum(zone_ra,2);
        n = nr + no + na;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rij_x = rij_x./r_matrix;
        rij_y = rij_y./r_matrix;
        rij_z = rij_z./r_matrix;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rij_zor_x = rij_x;
        rij_zor_x(~zone_rr) = 0;
        rij_zor_y = rij_y;
        rij_zor_y(~zone_rr) = 0;
        rij_zor_z = rij_z;
        rij_zor_z(~zone_rr) = 0;
        dr = -[sum(rij_zor_x,2),sum(rij_zor_y,2),sum(rij_zor_z,2)]';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        V_zoo_x = repmat(V(1,:),N,1);
        V_zoo_x(~zone_ro) = 0;
        V_zoo_y = repmat(V(2,:),N,1);
        V_zoo_y(~zone_ro) = 0;
        V_zoo_z = repmat(V(3,:),N,1);
        V_zoo_z(~zone_ro) = 0;
        do = [sum(V_zoo_x,2),sum(V_zoo_y,2),sum(V_zoo_z,2)]';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rij_zoa_x = rij_x;
        rij_zoa_x(~zone_ra) = 0;
        rij_zoa_y = rij_y;
        rij_zoa_y(~zone_ra) = 0;
        rij_zoa_z = rij_z;
        rij_zoa_z(~zone_ra) = 0;
        da = [sum(rij_zoa_x,2),sum(rij_zoa_y,2),sum(rij_zoa_z,2)]';


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        s1 = n == 0;
        s2 = nr == 0;
        s3 = s2 & no ~=0 & na ==0;
        s4 = s2 & no ==0 & na ~=0;
        s5 = s2 & no ~=0 & na ~=0;

        d = zeros(3,N);
        d([1;2;3],s1) = V([1;2;3],s1);
        d([1;2;3],~s2) = dr([1;2;3],~s2);
        d([1;2;3],s3) = do([1;2;3],s3);
        d([1;2;3],s4) = da([1;2;3],s4);
        d([1;2;3],s5) = 0.5*(do([1;2;3],s5) + da([1;2;3],s5));
        %==========================================================================
    end

    function V_new = add_noise_to_vel(V,sigma)
        %MY_V_ROTATE
        
        e = rand(size(V));
        e = e./vecnorm(e);
        N = size(V,2);
        phi = normrnd(0,repmat(sigma,1,N));    

        V_new = zeros(3,N);
        V_new(1,:) = sum((cosd(phi).*[1;0;0] + (1 - cosd(phi)).*(e.*e(1,:)) - sind(phi).* [zeros(1,N);e(3,:);-e(2,:)]).*V);
        V_new(2,:) = sum((cosd(phi).*[0;1;0] + (1 - cosd(phi)).*(e.*e(2,:)) - sind(phi).* [-e(3,:);zeros(1,N);e(1,:)]).*V);
        V_new(3,:) = sum((cosd(phi).*[0;0;1] + (1 - cosd(phi)).*(e.*e(3,:)) - sind(phi).* [e(2,:);-e(1,:);zeros(1,N)]).*V);
    end

    function y = my_saturation(x,min,max)
        %MY_SATURATION

        my_ismin = x<min;
        my_ismax = x>max;
        y = x.*((~my_ismax)&(~my_ismin)) + max * (my_ismax) + min*(my_ismin);
    end

end