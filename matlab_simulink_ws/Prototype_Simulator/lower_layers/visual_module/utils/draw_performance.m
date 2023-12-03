function draw_performance(my_axes_2, time_series, values_series)
%VISUAL_MODULE_DRAW_PERFORMANCE Summary of this function goes here
%   Detailed explanation goes here

origin_hold = ishold(my_axes_2);
if ~origin_hold
    hold(my_axes_2,"on");
end

num_values = size(values_series,1);

%%%
obj = findobj(my_axes_2.Children,'Tag','Value1');
flag_init = isempty(obj);
if flag_init
    for i = 1:num_values
        p = plot(my_axes_2, time_series,values_series(i,:),'LineWidth',2);
        p.Tag = strcat("Value",num2str(i));
    end
else
    for i = 1:num_values
        obj = findobj(my_axes_2.Children,'Tag',strcat("Value",num2str(i)));
        set(obj,'XData',time_series,'YData',values_series(i,:));
    end
end


if ~origin_hold
    hold(my_axes_2,"off");
end

end

