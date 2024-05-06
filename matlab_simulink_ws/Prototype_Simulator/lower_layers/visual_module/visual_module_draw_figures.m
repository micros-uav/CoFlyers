function visual_module_draw_figures(states, time_series_s, states_series, time_series, values_series,...
    map3d_faces, map3d_struct, model_stls, terrain, terrain_params, mode_simulation, flag_stage,...
    flag_motion_model, fig)
%VISUAL_MODULE_DRAW_FIGURES Summary of this function goes here
%   Detailed explanation goes here

persistent my_video

%%% Three stages of function calling%%% 
FLAG_INIT = 0;      % Before the iteration
FLAG_RUNNING = 1;   % When the iteration
FLAG_FINAL = 2;     % After the iteration

%%% Obtain the string of the current time to establish the name of output
%%% files %%%
data_save_dir_name = get_dir_name_from_mode(mode_simulation);
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));

%%%Get the parameters of the visualization module%
file_name_param = "visual_module_parameters";
[~,str_core] = get_multi_core_value();
fun_params = str2func(strcat(file_name_param,str_core));

[activate_plot,...
time_interval_plot,...
activate_trajectory,...
follow_agent,...
activate_save_figure,...
activate_save_video,...
dim_visual,...
time_interval_trajectory,...
video_speed,...
x_range,...
y_range,...
z_range_map,...
legend_name,...
font_size,...
font_size_sub,...
marker_size,...
background_color,...
activate_BD_1,...
len_arm,...
cmap_terrain,...
cmap_traj,...
T_end] = fun_params();

legend_names = strsplit(legend_name,'|||');

%%% Get the handles of the axes %%%
Tag_axis_1 = "axis_swarm";
Tag_axis_2 = "axis_evaluation";
a_1 = findobj(fig,"Tag",Tag_axis_1);
a_2 = findobj(fig,"Tag",Tag_axis_2);

%% Init. Callback when the simulation starts
if flag_stage == FLAG_INIT
    %%% Axes create or getting %%%
    if isempty(a_1)
        % Axes layout
        tiledlayout(fig,1,2);
        a_1 = nexttile;
        a_2 = nexttile;
        a_1.Tag = Tag_axis_1;
        a_2.Tag = Tag_axis_2;
        % Change the position and size of the figure
        change_figure_position(fig);
    else
        a_2 = findobj(fig,"Tag",Tag_axis_2);
    end
    
    %%% Axes1 init %%%
    hold(a_1,'on'); grid(a_1,'on'); box(a_1,'on')
    axis(a_1,'equal')
    xlabel(a_1,'X Position (m)');
    ylabel(a_1,'Y Position (m)');
    if dim_visual == 3
        zlabel(a_1,'Z Position (m)');
    end
    title(a_1,['Elapsed time ',num2str(time_series_s(end),"%.2f"),' s'])
    set(a_1,'FontSize',font_size,'FontName','Times New Roman');

    %%% Perspective does not follow the group %%%
    if ~follow_agent
        xlim(a_1,x_range);
        ylim(a_1,y_range);
        if dim_visual == 3
            zlim(a_1,z_range_map);
        end
    end
    
    %%% Background color of axis 1 %%%
    set(a_1,'color',background_color)

    %%% Draw environment %%%
    draw_environment(a_1, map3d_faces, map3d_struct, model_stls, terrain, terrain_params, dim_visual, cmap_terrain);
    if dim_visual==3
        view(a_1,[-45,30]);
        axis(a_1,"equal");                      % Set aspect ratio.
        axis(a_1,"vis3d");
        camlight(a_1);                          % Add a light
        lighting(a_1,"gouraud");                % Use decent lighting.
    end

    %%% Axes2 init %%%
    hold(a_2,'on'); grid(a_2,'on'); box(a_2,'on')
    xlabel(a_2,'Elapsed time (s)');
    ylabel(a_2,'Performances');
    set(a_2,'FontSize',font_size,'FontName','Times New Roman');

    %%% Create a video and open it %%%
    if activate_save_video
        my_video = VideoWriter([data_save_dir_name,'/videos/video_',time_now_string,'.avi']);
        my_video.FrameRate = video_speed;
        my_video.Quality = 75;
        open(my_video);
    end
end

%% Callback when the simulation starts or runs
if flag_stage == FLAG_INIT || flag_stage == FLAG_RUNNING
    %Draw in axes a_1
    if activate_trajectory
        t_now        = time_series_s(end);
        ind_tail = find(time_series_s > t_now - time_interval_trajectory, 1); % Index corresponding to trajectory tail
        if ~isempty(ind_tail)
            draw_trajectory(a_1,time_series_s(ind_tail:end), states_series(:,:,ind_tail:end), dim_visual, cmap_traj, T_end);
        end
    end

    color_s_bodies = zeros(3,size(states,2));

    draw_body(a_1, states, dim_visual, flag_motion_model, len_arm, color_s_bodies, marker_size);

    draw_environment(a_1, map3d_faces, map3d_struct, model_stls, terrain, terrain_params, dim_visual, cmap_terrain);
    
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
        if isa(a_1.Parent,"matlab.ui.Figure")
            % Running from APP
            A = getframe(a_1.Parent);
        elseif isa(a_1.Parent.Parent,"matlab.ui.Figure")
            % Running from Script
            A = getframe(a_1.Parent.Parent);
        else
            A = [];
        end
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

%% Functions
function change_figure_position(figure_in)
    position_in = figure_in.Position;
    scnsize_in = get(0,'ScreenSize');
    position_new_in = [scnsize_in(3)/2 - position_in(3),position_in(2),position_in(3)*2,position_in(4)];
    set(figure_in,'Position',position_new_in)
end

end