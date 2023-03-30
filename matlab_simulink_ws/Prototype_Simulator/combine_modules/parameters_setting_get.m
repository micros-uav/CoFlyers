function parameters_settings = parameters_setting_get(app, mode_simulation)
%GUI_PARAMETERS 
% Note that except the first 8 parameters, other parameters
% will be transferred to the parameters of other modules

if isempty(app)
    %%% Default settings
    number               = 100;
    time_max            = 200;
    flag_plot             = true;
    flag_plot_traj      = true;
    flag_save_data    = false;
    time_state_save  = 1;       % > 0
    flag_save_video  = false;
    video_speed       = 10;
    time_trajectory   = 20;
    flag_motion_model   = 0;    % 0: point-mass, 1: quadcopter, 2: point-mass-rotation
    flag_alg  = 0;                        % 0: Vasarhelyi, 1: Vasarhelyi+will, 2: Couzin (select flag_motion_model=2)
    flag_eva = 0;                         % 0: for Vasarhelyi and Vasarhelyi+will, 1: for Couzin

    flag_plot_follow = false;
    dimension_visual = 3;

    switch flag_motion_model
        case 0 %point-mass
            sample_time_motion  = 0.1;
            sample_time_control_u = sample_time_motion*1;
            sample_time_control_b = sample_time_motion*1;
            time_plot       = 1.0;
        case 1 %quadcopter
            sample_time_motion  = 0.0025;
            sample_time_control_u = sample_time_motion*40;
            sample_time_control_b = sample_time_motion*4;
            time_plot       = 1;
        case 2 %point-mass-rotation
            sample_time_motion  = 0.1;
            sample_time_control_u = sample_time_motion*1;
            sample_time_control_b = sample_time_motion*1;
            time_plot       = 1;
        otherwise
            sample_time_motion  = 0.1;
            sample_time_control_u = sample_time_motion*1;
            sample_time_control_b = sample_time_motion*1;
            time_plot       = 1.0;
    end

    if mode_simulation ==1
        flag_plot           = false;
        flag_save_data      = false;
        flag_save_video     = false;
    elseif mode_simulation ==2
        flag_plot           = false;
        flag_save_data      = false;
        flag_save_video     = false;
    end

    % When running batch processing, only save three time (start,
    % middle, end);
    if mode_simulation == 2 % batch processing
        time_state_save = floor(time_max/2);
    end
    
    %
    switch flag_alg
        case 0

        case 1

        case 2
            flag_plot_follow = true;
            flag_plot_traj = false;
    end
    % Deal error

else
    time_max             = app.SimulationtimessSpinner.Value;
    sample_time_motion   = app.MotionsSpinner.Value;
    sample_time_control_u  = app.UpperControlsEditField.Value;
    sample_time_control_b  = app.BottomControlsEditField.Value;
    number               = app.NumberofagentsSpinner.Value;
    time_plot            = app.PlottimeintervalSpinner.Value;
    time_trajectory      = app.TrajectorytimeSpinner.Value;
    time_state_save      = app.StatestimeintervalSpinner.Value;
    if strcmp(app.TrajectoriesSwitch.Value , 'Off')
        flag_plot_traj = false;
    else
        flag_plot_traj = true;
    end

    if strcmp(app.DatasaveSwitch.Value , 'Off')
        flag_save_data = false;
    else
        flag_save_data = true;
    end

    if strcmp(app.VideosaveSwitch.Value , 'Off')
        flag_save_video = false;
    else
        flag_save_video = true;
    end

    if strcmp(app.PlotSwitch.Value , 'Off')
        flag_plot = false;
    else
        flag_plot = true;
    end

    video_speed = app.FramerateSpinner.Value;


    if strcmp(app.AgentFollowSwitch.Value , 'Off')
        flag_plot_follow = false;
    else
        flag_plot_follow = true;
    end

    if strcmp(app.PlotDimensionDropDown.Value,"2D")
        dimension_visual = 2;
    else
        dimension_visual = 3;
    end
    
    alg_string = app.TypeDropDown.Value;
    alg_items = app.TypeDropDown.Items;
    for i = 1:length(alg_items)
        if strcmp(alg_items{i},alg_string)
            flag_motion_model = i-1;
            break
        end
    end

    alg_string = app.SwarmAlgorithmDropDown.Value;
    alg_items = app.SwarmAlgorithmDropDown.Items;
    %     alg_items = {'Vasarhelyi','Vasarhelyi_will','Couzin'};
    for i = 1:length(alg_items)
        if strcmp(alg_items{i},alg_string)
            flag_alg = i-1;
            break
        end
    end

    eva_string = app.EvaluationMetricsDropDown.Value;
    eva_items = app.EvaluationMetricsDropDown.Items;
    for i = 1:length(eva_items)
        if strcmp(eva_items{i},eva_string)
            flag_eva = i-1;
            break
        end
    end
end


parameters_settings = [
    mode_simulation;
    number;
    time_max;
    sample_time_motion;
    sample_time_control_u;
    sample_time_control_b;
    flag_save_data;
    time_state_save;
    flag_motion_model;
    flag_alg;
    flag_eva;
    flag_plot;
    time_plot;

    flag_plot_traj;
    flag_plot_follow;
    time_trajectory;
    flag_save_video;
    video_speed;
    dimension_visual];
end

