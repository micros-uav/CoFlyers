function visual_module_draw_figures(parameters_visual,parameters_motion,parameters_map,...
    states, time_series_s, states_series, time_series, values_series, data_swarm_for_visual,values_for_visual,...
    map_lines, mode_simulation, flag_stage,flag_motion_model,flag_alg,flag_eva)
%VISUAL_MODULE_DRAW_FIGURES Summary of this function goes here
%   Detailed explanation goes here
[flag_plot_traj,...
    flag_follow,...
    flag_save_data,...
    flag_save_video,...
    dim,...
    time_trajectory,...
    video_speed] =...
    visual_module_parameters_deal(parameters_visual);

num = size(states,2);

color_s_values = zeros(3,num);
legend_values = string(zeros(1,size(values_series,1)));
%% Get data from other modules
% Map module
[data] = map_module_get_params_for_visual(parameters_map);
x_range = data(1:2); y_range = data(3:4); z_range_map = data(5:6);

% Motion module
if flag_motion_model==1
    data_motion = motion_module_get_params_for_visual(parameters_motion,flag_motion_model);
    len_arm = data_motion*3;
else
    len_arm = [];
end

% Evaluation module
switch flag_eva
    case 0
        color_map = curl';
        angle_s = values_for_visual;
        color_s_values = color_map(:,ceil((angle_s+pi)/2/pi*256));
        legend_values = ["$\phi^{corr}$","$\phi^{vel}$","$\phi^{coll}$","$\phi^{wall}$","$\phi^{MND}$"];
    case 1
        legend_values = ["$p^{group}$","$g^{group}$"];
    otherwise
    legend_values = [];
end

% Swarm module
color_s_alg = zeros(3,num);
if flag_alg == 1
    will_s = data_swarm_for_visual;
    color_s_alg(1, will_s>0) = will_s(will_s>0);
    color_s_alg(2:3, will_s>0) = 0;
end


%%% Colors of the bodies pf the point-mass agents
color_s_bodies = color_s_values;
color_s_bodies(:, color_s_alg(1,:)>0.001) = color_s_alg(:,color_s_alg(1,:)>0.001);

%%%
persistent f_1 f_2 f_3 my_video


data_save_dir_name = get_dir_name_from_mode(mode_simulation);
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));

%% Callback when the simulation starts
flag_init = isempty(f_1) || (~isgraphics(f_1)) || flag_stage==0;
if flag_init
    f_1 = figure;
    f_2 = figure;
    if flag_save_video
        my_video = VideoWriter([data_save_dir_name,'/videos/video_',time_now_string,'.avi']);
        my_video.FrameRate = video_speed;
        my_video.Quality = 75;
        open(my_video);
    end

%     f_1.UserData.time = 0;
   
    %====Figure1, axes1 init====%
    a_1 = axes(f_1);
    a_1.Tag = "CF1";
    hold(a_1,'on'); grid(a_1,'on'); box(a_1,'on')
    axis(a_1,'equal')
    xlabel(a_1,'X Position (m)');
    ylabel(a_1,'Y Position (m)');
    if dim == 3
        zlabel(a_1,'Z Position (m)');
        view(a_1,[-45,30]);
    end
    title(a_1,['Elapsed time ',num2str(time_series(end),"%.2f"),' s'])
    % Colorbar
    if flag_plot_traj
        c = colorbar(a_1);
        c.Label.String = 'Speed (m/s)';
        caxis(a_1,[0,0.24]);
        c.FontSize = 14;
    end
    set(a_1,'FontSize',18,'FontName','Times New Roman');
    % Figure position
    position = f_1.Position;
    scnsize = get(0,'ScreenSize');
    position_new = [scnsize(3)/2 - position(3),position(2),position(3),position(4)];
    set(f_1,'Position',position_new)

     if ~flag_follow
        xlim(a_1,x_range);
        ylim(a_1,y_range);
        if dim == 3
            zlim(a_1,z_range_map);
        end
    
    end

    %====Figure2, axes2 init====%
    a_2 = axes(f_2);
    a_2.Tag = "CF2";
    hold(a_2,'on'); grid(a_2,'on'); box(a_2,'on')
    xlabel(a_2,'Elapsed time (s)');
    ylabel(a_2,'Performances');
    set(a_2,'FontSize',18,'FontName','Times New Roman');

    position = f_2.Position;
    scnsize = get(0,'ScreenSize');
    position_new = [scnsize(3)/2,position(2),position(3),position(4)];
    set(f_2,'Position',position_new);
    
else
    a_1 = findobj(f_1.Children,"Type","Axes","Tag","CF1");
    a_2 = findobj(f_2.Children,"Type","Axes","Tag","CF2");
end

%% Callback when the simulation starts or runs
%Draw in axes a_1
if flag_plot_traj
    t_now        = time_series_s(end);
    ind_tail = find(time_series_s > t_now - time_trajectory, 1); % Index corresponding to trajectory tail
    if ~isempty(ind_tail)
        draw_trajectory(a_1, states_series(:,:,ind_tail:end),dim);
    end
end

draw_body(a_1, states, dim, flag_motion_model, len_arm, color_s_bodies);

if flag_init
    draw_environment(a_1, map_lines, dim,  z_range_map)
end
a_1.Title.String = ['Elapsed time ',num2str(time_series(end),"%.2f"),' s'];

if flag_follow
    x_range = [min(states(1,:),[],"all"),max(states(1,:),[],"all")];
    y_range = [min(states(2,:),[],"all"),max(states(2,:),[],"all")];
    x_spin = x_range(2) - x_range(1);
    y_spin = y_range(2) - y_range(1);
    max_spin = max(x_spin/2,y_spin/2);
    x_range = [mean(x_range)-max_spin,mean(x_range)+max_spin];
    y_range = [mean(y_range)-max_spin,mean(y_range)+max_spin];  
    xlim(a_1,x_range);
    ylim(a_1,y_range);
    if dim == 3
        z_range = [min(states(3,:),[],"all"),max(states(3,:),[],"all")];
        z_spin = z_range(2) - z_range(1);
        max_spin = max(max_spin,z_spin/2);
        z_range = [mean(z_range)-max_spin,mean(z_range)+max_spin];  
        zlim(a_1,z_range);
    end
end

%Draw in axes a_2
draw_performance(a_2, time_series, values_series);
if flag_init
    legend(a_2,legend_values,'Interpreter','latex','Location','southeast');
end
drawnow;

if flag_save_video
    A = getframe(f_1);
    writeVideo(my_video,A);
end


%% Callback when the simulation ends
flag_final = flag_stage==2;
if flag_final

    f_3 = figure;
    %====Figure3, axes3 init====%
    a_3 = axes(f_3);
    a_3.Tag = "CF3";
    hold(a_3,'on'); grid(a_3,'on'); box(a_3,'on')
    axis(a_3,'equal')
    xlabel(a_3,'X Position (m)');
    ylabel(a_3,'Y Position (m)');
    if dim == 3
        zlabel(a_3,'Z Position (m)');
        view(a_3,[-45,30]);
    end
    
    xlim(a_3,x_range);
    ylim(a_3,y_range);
    if dim == 3
        zlim(a_3,z_range_map);
    end
    % Colorbar
    if flag_plot_traj
        c = colorbar(a_3);
        c.Label.String = 'Speed (m/s)';
        caxis(a_3,[0,0.24]);
        c.FontSize = 14;
    end
    set(a_3,'FontSize',18,'FontName','Times New Roman');
    %
    position = f_3.Position;
    scnsize = get(0,'ScreenSize');
    position_new = [scnsize(3)/2,position(2)-position(4)*1.2,position(3),position(4)];
    set(f_3,'Position',position_new);
    
    % Draw in axes a_3
    if flag_plot_traj
        states_series(:,:,end) = states;
        draw_trajectory(a_3, states_series, dim);
    end
    draw_body(a_3, states, dim, flag_motion_model, len_arm, color_s_bodies);
    draw_environment(a_3, map_lines, dim,  z_range_map)
    
    % Video
    if flag_save_video
        close(my_video);
    end

    % Save figrues
    if flag_save_data
        saveas(f_2,[data_save_dir_name,'/figures/values_',time_now_string],'png');
        saveas(f_3,[data_save_dir_name,'/figures/trajectories_',time_now_string],'png');
    end
end


end