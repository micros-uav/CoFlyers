close all
tic
time_max = 100;
sample_time_motion = 0.1;
flag_write_gif = 0;
[resolution,...
r_sense_min,...
r_sense_max,...
phi_range,...
psi_range] = lidar_module_parameters();

pos_agent = [2;3;1];
vel_agent = [0;0;0];
att_agent = [0;0;0];

[map3d_faces, map3d_struct, model_stls, ~, position0] = read_parameter_xml("xml_config_files\parameters_PSO.xml");

face_s_0 = generate_faces_from_stl("sphere_rough.stl", [0;0;0], [0;0;0], [0.1;0.1;0.1]);

% [range_s, psi_s, phi_s] = lidar_module_get_ranges(pos_agent, att_agent, map3d_faces);

[x_range, y_range, z_range] = map_module_get_range(map3d_struct);

dim_visual = 3;



f_1 = figure;
a_1 = gca;
axis equal;
grid on;
box on;
xlim(x_range)
ylim(y_range)
zlim(z_range)
view([15,60])
light("Style","local","Position",[-10 -10 40]);
xticklabels([])
yticklabels([])
zticklabels([])
count = 1;
for t = 0:sample_time_motion:time_max
    [map3d_faces,map3d_struct] =...
        map_module_update_map3d(t,sample_time_motion,map3d_faces,map3d_struct);
    
    [range_s, psi_s, phi_s] = lidar_module_get_ranges(pos_agent, att_agent, map3d_faces);
    V = range_s>r_sense_min & range_s<1;
    u_x = sum(-V.*cos(psi_s));
    u_y = sum(-V.*sin(psi_s));

%     draw_body(a_1,[pos_agent;vel_agent],3,0,1,[0;0;0]);
    obj = findobj(a_1.Children,'Tag','Body');
    flag_init = isempty(obj);
    face_s = face_s_0;
    face_s(1:3:end) = face_s(1:3:end)+pos_agent(1);
    face_s(2:3:end) = face_s(2:3:end)+pos_agent(2);
    face_s(3:3:end) = face_s(3:3:end)+pos_agent(3);
    if flag_init
        p = patch(a_1, 'XData',face_s(1:3:9,:),'YData',face_s(2:3:9,:),'ZData',face_s(3:3:9,:),...
            'FaceColor',[0;0;0],'LineStyle','none');
        p.Tag = 'Body';
    else
        set(obj,"XData",face_s(1:3:9,:),"YData",face_s(2:3:9,:),"ZData",face_s(3:3:9,:))
    end
        

    draw_Lidar(a_1, pos_agent, att_agent, psi_s,range_s,r_sense_max,r_sense_min,dim_visual);
    draw_environment(a_1, map3d_faces, map3d_struct, model_stls, dim_visual);
    
    vel_d = [u_x;u_y;0];
    vel_d_norm = norm(vel_d);
    if vel_d_norm == 0
        vel_norm = norm(vel_agent);
        if vel_norm == 0
            vel_d = [rand(2,1);0];
        else
            vel_d = vel_agent;
        end
    end
    vel_d = vel_d/norm(vel_d)*0.2;
%     vel_d = vel_d + [normrnd(0,0.02,[2,1]);0];

    acc = (vel_d - vel_agent)/sample_time_motion;
    acc_norm = norm(acc);
    if acc_norm > 0.5
        acc = acc/acc_norm*0.5;
    end
    vel_agent = vel_agent + acc*sample_time_motion;
    vel_norm = norm(vel_agent);
    if vel_norm > 0.2
        vel_agent = vel_agent/vel_norm*0.2;
    end
    pos_agent = pos_agent + vel_agent*sample_time_motion;
%     att_agent = [vel_agent(2);vel_agent(1);0]*180/pi;
    if flag_write_gif
        im_fig = getframe(f_1);
        im = frame2im(im_fig);
        im = imcrop(im,[105.5100   29.5100  368.9800  352.9800]);
        [A,map] = rgb2ind(im,256);
        if count == 1
            imwrite(A,map, 'animation.gif',...
                'gif', 'LoopCount', inf, 'DelayTime', 5/30);
        else
            if mod(count,4) == 1
                imwrite(A,map, 'animation.gif',...
                'gif',"WriteMode","append", 'DelayTime', 1/30);
            end
        end
    end
    drawnow;
    count = count+1;
end
toc