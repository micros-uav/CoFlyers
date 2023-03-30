function commands_bottom = pmr_control(states,xyz_ds, vxyz_d_fws, parameters_motion,control_mode,sample_time_control)
%PM_CONTROL_BY_POSVEL_2D 

%    0b000001: add position control
%    0b000010: add speed feedforword and angle
%    0b001000: height maintenance

%%% Get paramters
[speed_max,...
    theta_max,...
    time_constant_pos_ctrl] = pmr_parameters_deal(parameters_motion);

if bitand(control_mode,0b001000) == 0
    n = size(states,2);
    % 3D rotation model
    %%% Calculate desired velocity
    vxyz_ds = (xyz_ds - states(1:3,:))/time_constant_pos_ctrl;
    vxyz_ds(:,bitand(control_mode,0b000001) == 0) = 0;
    vxyz_d_fws(:,bitand(control_mode,0b000010) == 0) = 0;
    vxyz_ds = vxyz_ds + vxyz_d_fws;

%     speed_ds = sqrt(sum(vxyz_ds.^2,1));
    speed_ds = vecnorm(vxyz_ds,2,1);
    clamp_v_d = find(speed_ds >  speed_max);
    if ~isempty(clamp_v_d)
        vxyz_ds(:,clamp_v_d) = vxyz_ds(:,clamp_v_d)./speed_ds(clamp_v_d) * speed_max;
        speed_ds(clamp_v_d) = speed_max;
    end
    %%% Rotation
    vxyz_s = states(4:6,:);
    speed_s = vecnorm(vxyz_s,2,1);
    temp = cross(vxyz_s,vxyz_ds);
    rot_norm = vecnorm(temp,2,1);
    rot_norm(rot_norm==0) = 1e-16;
    rot_dir_s      = temp./rot_norm;

    temp = dot(vxyz_s,vxyz_ds)./(speed_s.*speed_ds);
    temp(temp>1) = 1;
    temp(temp<-1)=-1;
    theta_s =  acos(temp);
    
    theta_s(theta_s>theta_max*sample_time_control) =  theta_max*sample_time_control;
    

    cos_theta_s = reshape(cos(theta_s),1,1,n);
    v_1 = reshape(rot_dir_s,[3,1,n]);
    
    skew_R = [zeros(1,1,n),-v_1(3,1,:),v_1(2,1,:);
        v_1(3,1,:), zeros(1,1,n), -v_1(1,1,:);
        -v_1(2,1,:),v_1(1,1,:),zeros(1,1,n)];
    R_s = eye(3).*cos_theta_s+ ...
        (1 -cos_theta_s) .* pagemtimes(v_1,reshape(rot_dir_s,[1,3,n])) + ...
        reshape(sin(theta_s),1,1,n) .* skew_R;
    
    vxyz_ds = squeeze(pagemtimes(R_s,reshape(vxyz_s,[3,1,n])))./speed_s.*speed_ds;
else
    n = size(states,2);
    % horizontal rotation model, height maintenance
    %%% Calculate desired velocity
    if bitand(control_mode,0b000001) == 0
        % No horizontal position control
        vxyz_ds = zeros(3,n);
        vxyz_ds(3,:) = (xyz_ds(3,:) - states(3,:))/time_constant_pos_ctrl;
    else
        vxyz_ds = (xyz_ds - states(1:3,:))/time_constant_pos_ctrl;
    end

    if bitand(control_mode,0b000010) > 0
        vxyz_ds = vxyz_ds + vxyz_d_fws;
    end
    
    speed_ds = vecnorm(vxyz_ds,2,1);
    clamp_v_d = find(speed_ds >  speed_max);
    if ~isempty(clamp_v_d)
        vxyz_ds(:,clamp_v_d) = vxyz_ds(:,clamp_v_d)./speed_ds(clamp_v_d) * speed_max;
    end
    %%% Rotation
    vx = states(4,:);
    vy = states(5,:);
    vx_d = vxyz_ds(1,:);
    vy_d = vxyz_ds(2,:);
    speed_h_s = sqrt(vx.^2 + vy.^2);
    speed_h_d_s = sqrt(vx_d.^2 + vy_d.^2);
    z_cross = vx.*vy_d - vy.*vx_d;
    temp = (vx.*vx_d + vy.*vy_d)./speed_h_d_s./speed_h_s;
    temp(temp>1) = 1;
    temp(temp<-1) = -1;
    theta_s = acos(temp);
    theta_s(theta_s>theta_max*sample_time_control) =  theta_max*sample_time_control;
    theta_s = sign(z_cross).*theta_s;

    cos_theta_s = cos(theta_s);
    sin_theta_s = sin(theta_s);
    
    vxyz_ds(1,:) = cos_theta_s.*vx - sin_theta_s.*vy;
    vxyz_ds(2,:) = sin_theta_s.*vx + cos_theta_s.*vy;
    vxyz_ds(1:2,:) = vxyz_ds(1:2,:)./speed_h_s.*speed_h_d_s;
end

commands_bottom = vxyz_ds;

end

