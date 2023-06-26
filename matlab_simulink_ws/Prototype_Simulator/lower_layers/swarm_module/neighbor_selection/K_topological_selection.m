function ind_s = K_topological_selection(dis_to_neighbors, k)
%K_TOPOLOGICAL_SELECTION Summary of this function goes here
%   Detailed explanation goes here
[~,ind_s] = mink(dis_to_neighbors,k);
end

