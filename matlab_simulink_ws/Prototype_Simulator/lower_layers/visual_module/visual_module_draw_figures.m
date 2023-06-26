function visual_module_draw_figures(states, time_series_s, states_series, time_series, values_series,...
    map3d_faces, map3d_struct, model_stls, mode_simulation, flag_stage,flag_motion_model,avtivate_fig1,activate_fig2,activate_fig3)
%VISUAL_MODULE_DRAW_FIGURES Summary of this function goes here
%   Detailed explanation goes here

if nargin < 12
    avtivate_fig1 = true;
    activate_fig2 = true;
    activate_fig3 = true;
end

% For simulink
if isempty(model_stls)
    model_stls = importdata("models_stls.mat");
end

%
[activate_plot,...
time_interval_plot,...
activate_trajectory,...
follow_agent,...
activate_save_figure,...
activate_save_video,...
dim_visual,...
time_interval_trajectory,...
video_speed] = visual_module_parameters();

legend_name = ["$\phi^{corr}$","$\phi^{vel}$","$\phi^{coll}$","$\phi^{wall}$","$\phi^{MND}$"];

num = size(states,2);

%% Get data from other modules
% Map module
[x_range, y_range, z_range_map] = map_module_get_range(map3d_struct);
if isempty(x_range)
    x_range = [0,1];
end
if isempty(y_range)
    y_range = [0,1];
end
if isempty(z_range_map)
    z_range_map = [0,1];
end

%%% Colors of the bodies pf the point-mass agents
color_s_bodies = zeros(3,num);

%%%
persistent f_1 f_2 f_3 my_video


data_save_dir_name = get_dir_name_from_mode(mode_simulation);
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));

%% Callback when the simulation starts
flag_init = isempty(f_1) || (~isgraphics(f_1)) || flag_stage==0;
if flag_init
    if activate_save_video
        my_video = VideoWriter([data_save_dir_name,'/videos/video_',time_now_string,'.avi']);
        my_video.FrameRate = video_speed;
        my_video.Quality = 75;
        open(my_video);
    end
    if avtivate_fig1
        %====Figure1, axes1 init====%
        f_1 = figure;
        a_1 = axes(f_1);
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
            c.FontSize = 14;
        end
        set(a_1,'FontSize',18,'FontName','Times New Roman');
        % Figure position
        position = f_1.Position;
        scnsize = get(0,'ScreenSize');
        position_new = [scnsize(3)/2 - position(3),position(2),position(3),position(4)];
        set(f_1,'Position',position_new)

        if ~follow_agent
            xlim(a_1,x_range);
            ylim(a_1,y_range);
            if dim_visual == 3
                zlim(a_1,z_range_map);
            end

        end

        draw_environment(a_1, map3d_faces, map3d_struct, model_stls, dim_visual);
    end
    %====Figure2, axes2 init====%
    if activate_fig2
        f_2 = figure;
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
    end


else
    a_1 = findobj(f_1.Children,"Type","Axes","Tag","CF1");
    a_2 = findobj(f_2.Children,"Type","Axes","Tag","CF2");
end

%% Callback when the simulation starts or runs
if avtivate_fig1
    %Draw in axes a_1
    if activate_trajectory
        t_now        = time_series_s(end);
        ind_tail = find(time_series_s > t_now - time_interval_trajectory, 1); % Index corresponding to trajectory tail
        if ~isempty(ind_tail)
            draw_trajectory(a_1, states_series(:,:,ind_tail:end),dim_visual);
        end
    end

    len_arm = 0.1;
    draw_body(a_1, states, dim_visual, flag_motion_model, len_arm, color_s_bodies);

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
end
if activate_fig2
    %Draw in axes a_2
    draw_performance(a_2, time_series, values_series);
    if flag_init
        leg = legend(a_2,legend_name,'Interpreter','latex','Location','southeast',"AutoUpdate","off");
    end
    drawnow;

    if activate_save_video
        A = getframe(f_1);
        writeVideo(my_video,A);
    end
end

%% Callback when the simulation ends
flag_final = flag_stage==2;
if flag_final 
    if activate_fig3
        f_3 = figure;
        %====Figure3, axes3 init====%
        a_3 = axes(f_3);
        a_3.Tag = "CF3";
        hold(a_3,'on'); grid(a_3,'on'); box(a_3,'on')
        axis(a_3,'equal')
        xlabel(a_3,'X Position (m)');
        ylabel(a_3,'Y Position (m)');
        if dim_visual == 3
            zlabel(a_3,'Z Position (m)');
            view(a_3,[-45,30]);
        end

        xlim(a_3,x_range);
        ylim(a_3,y_range);
        if dim_visual == 3
            zlim(a_3,z_range_map);
        end
        % Colorbar
        if activate_trajectory
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
        if activate_trajectory
            states_series(:,:,end) = states;
            draw_trajectory(a_3, states_series, dim_visual);
        end
        draw_body(a_3, states, dim_visual, flag_motion_model, len_arm, color_s_bodies);

        draw_environment(a_3, map3d_faces, map3d_struct, model_stls, dim_visual);
    end

    % Video
    if activate_save_video
        close(my_video);
    end

    % Save figrues
    if activate_save_figure
        if activate_fig2
            saveas(f_2,[data_save_dir_name,'/figures/values_',time_now_string],'png');
        end
        if activate_fig3
            saveas(f_3,[data_save_dir_name,'/figures/trajectories_',time_now_string],'png');
        end
    end
end


end