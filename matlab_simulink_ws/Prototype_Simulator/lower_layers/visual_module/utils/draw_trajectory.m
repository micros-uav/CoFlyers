function draw_trajectory(my_axes_1, states_series, dim)
%VISUAL_MODULE_DRAW_TRAJECTORY Summary of this function goes here
% my_axes_1: target axes
% states_series: states of agents, [num_states, num_agents, num_times] 
% dim: dimention. 3 or 2.

% when the function is called for the first time, make
% flag_init=true and the function will init the trajectories of my_axes_1. 
% Otherwise, make it 0 and the function will update the trajectories data
% of my_axes_1. 

number = size(states_series,2);
num_t = size(states_series,3);

obj = findobj(my_axes_1.Children,'Tag','Trajectory');
flag_init = isempty(obj);

if isempty(states_series)
    states_series = ones(6,6,1);
end

% Change the data format to fit the input of the 'patch' function
xs = reshape(states_series(1,:,:),[number,num_t])'; xs(end,:) = nan;
ys = reshape(states_series(2,:,:),[number,num_t])';
if dim == 3
    cs = reshape(sum(sqrt(states_series(4:6,:,:).^2),1),[number,num_t])';
    zs = reshape(states_series(3,:,:),[number,num_t])';
    trajectories = cat(3,xs,ys,zs,cs);
elseif dim == 2
    cs = reshape(sum(sqrt(states_series(4:5,:,:).^2),1),[number,num_t])';
    trajectories = cat(3,xs,ys,cs);
end

if size(trajectories,1) < 3
    trajectories = nan*trajectories;
end

if flag_init
    %====Initialize and label the image object of trajectories=====%
    if dim == 3
        p = patch(my_axes_1,trajectories(:,:,1),trajectories(:,:,2),trajectories(:,:,3),trajectories(:,:,4),...
            'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    elseif dim == 2
        p = patch(my_axes_1,trajectories(:,:,1),trajectories(:,:,2),trajectories(:,:,3),...
            'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    end
    p.Tag = 'Trajectory';
else
    %====Update data====%
    if dim == 3
        set(obj,'XData',trajectories(:,:,1),'YData',trajectories(:,:,2),'ZData',trajectories(:,:,3),'CData',trajectories(:,:,4));
    elseif dim == 2
        set(obj,'XData',trajectories(:,:,1),'YData',trajectories(:,:,2),'CData',trajectories(:,:,3));
    end
end

end

