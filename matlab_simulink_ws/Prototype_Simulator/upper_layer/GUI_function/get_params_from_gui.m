function parameters_gui = get_params_from_gui(app)
%GET_PARAMS_FROM_GUI Summary of this function goes here
%   Detailed explanation goes here


if ~isempty(app)
    time_max             = app.SimulationtimessSpinner.Value;
    sample_time_motion   = app.MotionsSpinner.Value;
    sample_time_control_upper  = app.UpperControlsEditField.Value;
    sample_time_control_bottom  = app.BottomControlsEditField.Value;
    number               = app.NumberofagentsSpinner.Value;
    time_interval_plot            = app.PlottimeintervalSpinner.Value;
    time_interval_trajectory      = app.TrajectorytimeSpinner.Value;
    time_interval_save      = app.StatestimeintervalSpinner.Value;
    if strcmp(app.TrajectoriesSwitch.Value , 'Off')
        activate_trajectory = false;
    else
        activate_trajectory = true;
    end

    if strcmp(app.DatasaveSwitch.Value , 'Off')
        activate_save_states = false;
    else
        activate_save_states = true;
    end
    

    if strcmp(app.FiguresaveSwitch.Value , 'Off')
        activate_save_figure = false;
    else
        activate_save_figure = true;
    end

    if strcmp(app.VideosaveSwitch.Value , 'Off')
        activate_save_video = false;
    else
        activate_save_video = true;
    end

    if strcmp(app.PlotSwitch.Value , 'Off')
        activate_plot = false;
    else
        activate_plot = true;
    end

    video_speed = app.FramerateSpinner.Value;


    if strcmp(app.AgentFollowSwitch.Value , 'Off')
        follow_agent = false;
    else
        follow_agent = true;
    end

    if strcmp(app.PlotDimensionDropDown.Value,"2D")
        dim_visual = 2;
    else
        dim_visual = 3;
    end

    alg_string = app.TypeDropDown.Value;
    alg_items = app.TypeDropDown.Items;
    for i = 1:length(alg_items)
        if strcmp(alg_items{i},alg_string)
            motion_model_type = i-1;
            break
        end
    end

%     alg_string = app.SwarmAlgorithmDropDown.Value;
%     alg_items = app.SwarmAlgorithmDropDown.Items;
%     %     alg_items = {'Vasarhelyi','Vasarhelyi_will','Couzin'};
%     for i = 1:length(alg_items)
%         if strcmp(alg_items{i},alg_string)
%             swarm_algorithm_type = i-1;
%             break
%         end
%     end
    swarm_algorithm_type = app.SwarmAlgorithmDropDown.Value;

%     eva_string = app.EvaluationMetricsDropDown.Value;
%     eva_items = app.EvaluationMetricsDropDown.Items;
%     for i = 1:length(eva_items)
%         if strcmp(eva_items{i},eva_string)
%             evaluation_metric_type = i-1;
%             break
%         end
%     end
    evaluation_metric_type = app.EvaluationMetricsDropDown.Value;

    parameters_settings = struct();
    parameters_settings.number = number;
    parameters_settings.time_max = time_max;
    parameters_settings.sample_time_motion = sample_time_motion;
    parameters_settings.sample_time_control_upper = sample_time_control_upper;
    parameters_settings.sample_time_control_bottom = sample_time_control_bottom;
    parameters_settings.activate_save_states = activate_save_states;
    parameters_settings.time_interval_save = time_interval_save;
    parameters_settings.motion_model_type = motion_model_type;
    parameters_settings.swarm_algorithm_type = swarm_algorithm_type;
    parameters_settings.evaluation_metric_type = evaluation_metric_type;
%     field_names = fieldnames(parameters_settings);
%     for i = 1:length(field_names)
%         parameters_settings.(field_names{i}) = double(parameters_settings.(field_names{i}));
%     end


    parameters_visual = struct();
    parameters_visual.activate_plot = activate_plot;
    parameters_visual.time_interval_plot = time_interval_plot;
    parameters_visual.activate_trajectory = activate_trajectory;
    parameters_visual.follow_agent = follow_agent;
    parameters_visual.activate_save_figure = activate_save_figure;
    parameters_visual.activate_save_video = activate_save_video;
    parameters_visual.dim_visual = dim_visual;
    parameters_visual.time_interval_trajectory = time_interval_trajectory;
    parameters_visual.video_speed = video_speed;
%     field_names = fieldnames(parameters_visual);
%     for i = 1:length(field_names)
%         parameters_visual.(field_names{i}) = double(parameters_visual.(field_names{i}));
%     end
else
    parameters_settings = struct();
    parameters_visual = struct();
end

parameters_gui.parameters_settings = parameters_settings; 
parameters_gui.parameters_visual = parameters_visual;

end

