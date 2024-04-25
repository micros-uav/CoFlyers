function draw_trajectory(my_axes_1, time_series, states_series, dim, cmap, T_end)
%VISUAL_MODULE_DRAW_TRAJECTORY Summary of this function goes here
% my_axes_1: target axes
% states_series: states of agents, [num_states, num_agents, num_times] 
% dim: dimention. 3 or 2.

% when the function is called for the first time, make
% flag_init=true and the function will init the trajectories of my_axes_1. 
% Otherwise, make it 0 and the function will update the trajectories data
% of my_axes_1. 

%%% check input%%%
[~,number,num_t] = size(states_series);
if num_t < 3
    time_series = [time_series,repmat(time_series(end),1,3-num_t)];
    states_series = cat(3,states_series,repmat(states_series(:,:,end),1,1,3-num_t));
    num_t=3;
end
if nargin < 5
    cmap = hsv;
    T_end = max(200,time_series(end));
end

%%% Get the handle of the swarm trajectory%%%
obj = findobj(my_axes_1.Children,'Tag','Trajectory');
flag_init = isempty(obj);

if isempty(states_series)
    states_series = ones(6,6,1);
end

% Change the data format to fit the input of the 'patch' function
xs = reshape(states_series(1,:,:),[number,num_t])'; xs(end,:) = nan;
ys = reshape(states_series(2,:,:),[number,num_t])';
num_cmap = size(cmap,1);
temp1 = floor(time_series/T_end*(num_cmap-1))+1;
temp2 = reshape(cmap(temp1,:),num_t,1,3);
cs = repmat(temp2,1,number,1);
if dim == 3
    zs = reshape(states_series(3,:,:),[number,num_t])';
end

if flag_init
    %====Initialize and label the image object of trajectories=====%
    if dim == 3
        p = patch(my_axes_1,xs,ys,zs,cs,...
            'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    elseif dim == 2
        p = patch(my_axes_1,xs,ys,cs,...
            'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    end
    p.Tag = 'Trajectory';
else
    %====Update data====%
    if dim == 3
        set(obj,'XData',xs,'YData',ys,'ZData',zs,'CData',cs);
    elseif dim == 2
        set(obj,'XData',xs,'YData',ys,'CData',cs);
    end
end

end

