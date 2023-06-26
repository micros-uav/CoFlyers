close all;
clc
%%
number = 10;
r_agent = 0.5;

map_3d_structed =[[0;0;1;0.5;0],[0;1.5;1;0.5;0],[0;-1.5;1;0.5;0]];
time_max = 50;
delta_t = 0.1;
T_c = 1;

v_flock = 0.2;
r_rep = 0.5;
p_rep = 0.5;

% [map3d_struct, model_stls] = read_map_xml();
% [map3d_faces, map3d_grid] = generate_map3d_from_struct(map3d_struct, model_stls);
[map3d_faces, map3d_struct, model_stls] = map_module_generate_map3d();
[x_range,y_range,z_range] = map_module_get_range(map3d_struct);

r_sense =  0.4;
resolution_v = pi/20;
% phi_range = [0;0];
phi_range = [-pi;pi];
psi_range = [-pi;pi];
C = 2;

flag_gif = false;
delta_t_gif = 0.5;
%% initialize
position_s = zeros(3,number);
position_s(1,:) = rand(1,number)*2-1;
position_s(2,:) = rand(1,number) + y_range(1);
position_s(3,:) = rand(1,number) - 0.5 + z_range(1) + 1.5;
velocity_s = zeros(3,number) + 1e-10;
velocity_d_s = velocity_s;
% figure;
[x_sphere_unit,y_sphere_unit,z_sphere_unit] = sphere;



%
f1 = figure;
my_axis = gca;
hold on;
axis equal;
box on;
grid on;
view([45,40])
xlim(x_range);
ylim(y_range);
zlim(z_range);
% patch(map3d_faces(1:3:9,:),map3d_faces(2:3:9,:),map3d_faces(3:3:9,:),'r','FaceAlpha',0.5,'LineStyle','none');
draw_environment(my_axis, map3d_faces, map3d_struct,model_stls,3);
s = scatter3(position_s(1,:),position_s(2,:),position_s(3,:),'MarkerFaceColor',[0 .75 .75]);


if flag_gif
    filename = "test.gif";
    f = getframe(f1).cdata;
    [A,map] = rgb2ind(f,256);
    imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",0.1);
end
%%

for t = 0:delta_t:time_max
    for id = 1:number
        pos_id = position_s(:,id);
        pos_neighbor = position_s(:,[1:id-1,id+1:number]);
        %=====================vWall=======================%
        [range_s,psi_s,phi_s] = get_lidar_from_map3d(pos_id, [0;0;0], map3d_faces, resolution_v, r_sense, phi_range, psi_range);
        V = range_s < inf;
        u_x = -sum(cos(phi_s).*cos(psi_s).*V,'all')*(resolution_v)^2;
        u_y = -sum(cos(phi_s).*sin(psi_s).*V,'all')*(resolution_v)^2;
        u_z = -sum(sin(phi_s).*V,'all')*(pi/20)^2;
        v_wall_id = C*[u_x;u_y;u_z];
        %=====================vFlock=======================%
%         v_flock_id = velocity_s(:,id)/norm(velocity_s(:,id))*v_flock;
        v_flock_id = mean(velocity_s,2);
        v_flock_id = v_flock_id/norm(v_flock_id)*v_flock;
        %=====================vRep=======================%
        pos_ij = pos_neighbor - pos_id;
        pos_ij_norm =vecnorm(pos_ij);
        ind_in_rep = find(pos_ij_norm<r_rep);
        v_rep_id = p_rep * sum( (pos_ij_norm(ind_in_rep) - r_rep) .* pos_ij(:,ind_in_rep)./pos_ij_norm(ind_in_rep) ,2);
        %=====================vD=======================%
        v_d_id = v_wall_id + v_flock_id + v_rep_id;
        if norm(v_d_id) > 0.2
            v_d_id = v_d_id/norm(v_d_id)*0.2;
        end
        velocity_d_s(:,id) = v_d_id;
%         velocity_d_s(3,id) = 0;
    end
    % motion
    acceleration_s = (velocity_d_s - velocity_s)/T_c;
    velocity_s = velocity_s + acceleration_s*delta_t;
    position_s = position_s + velocity_s*delta_t;
    % update map
    [map3d_faces, map3d_struct] = map_module_update_map3d(t, delta_t, map3d_faces,map3d_struct);
    % draw
    draw_environment(my_axis, map3d_faces, map3d_struct,model_stls,3);
    set(s,'XData',position_s(1,:),'YData',position_s(2,:),'ZData',position_s(3,:))
    title(['Time: ',num2str(t),' s'])
    drawnow;
    if flag_gif
        if mod(t,delta_t_gif) == 0
            f = getframe(f1).cdata;
            [A,map] = rgb2ind(f,256);
            imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",0.01);
        end
    end
end
