function ind_s = FOV_selection(pos_to_neighbors_unit, vel_id_unit, alpha)
%FOV_SELECTION Summary of this function goes here
%   alpha: half FOV

temp = sum(pos_to_neighbors_unit.*vel_id_unit,1);
temp(temp>1)=1;
temp(temp<-1)=-1;
angle_view = acos(temp);
ind_s = find(angle_view < alpha);

end

