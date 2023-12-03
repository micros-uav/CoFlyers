function visual_module_draw_figures(states, time_series_s, states_series, time_series, values_series,...
    map3d_faces, map3d_struct, model_stls, mode_simulation, flag_stage,...
    flag_motion_model, fig, axis_1, axis_2)
%VISUAL_MODULE_DRAW_FIGURES Summary of this function goes here
%   Detailed explanation goes here

persistent a_1 a_2
persistent my_video

num = size(states,2);

FLAG_INIT = 0; FLAG_RUNNING = 1; FLAG_FINAL = 2;

% % For simulink
% if isempty(model_stls)
%     model_stls = importdata("models_stls.mat");
% end

% Obtain the string of the current time to establish the name of output files
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));

%=============================Get the parameters==========================%
file_name_param = "visual_module_parameters";
[~,str_core] = get_multi_core_value();
fun_params = str2func(strcat(file_name_param,str_core));

[activate_plot,time_interval_plot,activate_trajectory,...
follow_agent,activate_save_figure,activate_save_video,...
dim_visual,time_interval_trajectory,video_speed,...
x_range,y_range,z_range_map,legend_name,...
font_size,font_size_sub,marker_size,background_color] = fun_params();

legend_names = strsplit(legend_name,'|||');
%=========================================================================%


%%% Colors of the bodies pf the point-mass agents
colormap_body = hsv;
if dim_visual == 3
    vel_mean = mean(states(4:6,:),2);
    vel_mean = vel_mean/(norm(vel_mean)+1e-8);
    vel_unit = states(4:6,:)./(vecnorm(states(4:6,:))+1e-8);
    dangle_s = acos(sum(vel_unit.*vel_mean,1));
    dangle_s_int = floor(dangle_s/pi*256)+1;
    color_s_bodies = colormap_body(dangle_s_int,:)';
else
    direction = (atan2(states(5,:),states(4,:))+pi)/2/pi;
    direction_int = floor(direction*256)+1;
    color_s_bodies = colormap_body(direction_int,:)';
end
%% Callback when the simulation starts
if flag_stage == FLAG_INIT
    if activate_save_video
        my_video = VideoWriter([data_save_dir_name,'/videos/video_',time_now_string,'.avi']);
        my_video.FrameRate = video_speed;
        my_video.Quality = 75;
        open(my_video);
    end
    %====Axes create====%
    % if isempty(axis_1)
    %     change_figure_position(fig);
    %     tiledlayout(1,2);
    %     a_1 = nexttile;
    %     a_2 = nexttile;
    % else
    %     a_1 = axis_1;
    %     a_2 = axis_2;
    % end
    % change_figure_position(fig);
    a_1 = axis_1;
    a_2 = axis_2;
    


    %====Axes1 init====%
    a_1.Tag = "CF1";
    hold(a_1,'on'); grid(a_1,'on'); box(a_1,'on')
    axis(a_1,'equal')
    xlabel(a_1,'X Position (m)');
    ylabel(a_1,'Y Position (m)');
    if dim_visual == 3
        zlabel(a_1,'Z Position (m)');
        view(a_1,[-45,30]);
    end
    title(a_1,['Elapsed time ',num2str(time_series_s(end),"%.2f"),' s'])
    % Colorbar
    if activate_trajectory
        c = colorbar(a_1);
        c.Label.String = 'Speed (m/s)';
        caxis(a_1,[0,0.24]);
        c.FontSize = font_size_sub;
    end
    set(a_1,'FontSize',font_size,'FontName','Times New Roman');

    if ~follow_agent
        xlim(a_1,x_range);
        ylim(a_1,y_range);
        if dim_visual == 3
            zlim(a_1,z_range_map);
        end

    end
    
    set(a_1,'color',background_color)

    draw_environment(a_1, map3d_faces, map3d_struct, model_stls, dim_visual);
    %====Axes2 init====%
    a_2.Tag = "CF2";
    hold(a_2,'on'); grid(a_2,'on'); box(a_2,'on')
    xlabel(a_2,'Elapsed time (s)');
    ylabel(a_2,'Performances');
    set(a_2,'FontSize',font_size,'FontName','Times New Roman');
end

%% Callback when the simulation starts or runs
if flag_stage == FLAG_INIT || flag_stage == FLAG_RUNNING
    %Draw in axes a_1
    if activate_trajectory
        t_now        = time_series_s(end);
        ind_tail = find(time_series_s > t_now - time_interval_trajectory, 1); % Index corresponding to trajectory tail
        if ~isempty(ind_tail)
            draw_trajectory(a_1, states_series(:,:,ind_tail:end),dim_visual);
        end
    end

    len_arm = 0.1;
    draw_body(a_1, states, dim_visual, flag_motion_model, len_arm, color_s_bodies, marker_size);

    draw_environment(a_1, map3d_faces, map3d_struct, model_stls, dim_visual);

    a_1.Title.String = ['Elapsed time ',num2str(time_series(end),"%.2f"),' s'];

    if follow_agent
        x_range = [min(states(1,:),[],"all"),max(states(1,:),[],"all")];
        y_range = [min(states(2,:),[],"all"),max(states(2,:),[],"all")];
        x_spin = x_range(2) - x_range(1);
        y_spin = y_range(2) - y_range(1);
        max_spin = max(x_spin/2,y_spin/2);
        x_range = [mean(x_range)-max_spin,mean(x_range)+max_spin];
        y_range = [mean(y_range)-max_spin,mean(y_range)+max_spin];
        xlim(a_1,x_range);
        ylim(a_1,y_range);
        if dim_visual == 3
            z_range = [min(states(3,:),[],"all"),max(states(3,:),[],"all")];
            z_spin = z_range(2) - z_range(1);
            max_spin = max(max_spin,z_spin/2);
            z_range = [mean(z_range)-max_spin,mean(z_range)+max_spin];
            zlim(a_1,z_range);
        end
    end

    %Draw in axes a_2
    draw_performance(a_2, time_series, values_series);
    if flag_stage == FLAG_INIT
        % leg = legend(a_2,legend_name,'Interpreter','latex','Location','southeast',"AutoUpdate","off");
        leg = legend(a_2,legend_names,'Interpreter','latex',"AutoUpdate","off",'Location','southeastoutside');
    end
    drawnow;
    if activate_save_video
        A = getframe(a_1.Parent.Parent);
        writeVideo(my_video,A);
    end
end
%% Callback when the simulation ends
if flag_stage == FLAG_FINAL

    % Save video
    if activate_save_video
        close(my_video);
    end

    % Save figrues
    if activate_save_figure
        saveas(a_1.Parent,[data_save_dir_name,'/figures/values_',time_now_string],'png');
    end
end


end