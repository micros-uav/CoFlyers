function ind_s = Metric_selection(dis_to_neighbors, r)
%METRIC_SELECTION Summary of this function goes here
%   Detailed explanation goes here
ind_s = find(dis_to_neighbors<r);
end

