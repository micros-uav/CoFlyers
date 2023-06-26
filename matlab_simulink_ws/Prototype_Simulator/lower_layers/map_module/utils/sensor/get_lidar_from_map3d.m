function [range_s,psi_s,phi_s] = get_lidar_from_map3d(pos_agent, att_agent, face_s, resolution, r_min,  r_sense, phi_range, psi_range)
%GET_LIDAR_FROM_MAP3D
% att_agent: degree

% Make 'pos_agent' as the origin
face_s_temp = move_face_s(face_s, -pos_agent);

% Get faces within distance 'r_sense'
in_local = distance2faces([0;0;0], face_s_temp) < r_sense;
face_s_local = face_s_temp(:,in_local);


n_dangle = max(ceil((psi_range(2)-psi_range(1))/resolution), 1);
psi_1 = linspace(psi_range(1),psi_range(2),n_dangle);
n_dangle = max(ceil((phi_range(2)-phi_range(1))/resolution), 1);
phi_1 = linspace(phi_range(1),phi_range(2),n_dangle);
[psi_s,phi_s] = meshgrid(psi_1,phi_1);
if sum(in_local) > 0
    % Rotation
    t1 = att_agent(1);
    t2 = att_agent(2);
    t3 = att_agent(3);
    r_z = [cosd(t3),-sind(t3),0
        sind(t3),cosd(t3),0
        0,0,1];
    r_x = [1,0,0
        0,cosd(t1),-sind(t1)
        0,sind(t1),cosd(t1)];
    r_y = [cosd(t2),0,sind(t2)
        0,1,0
        -sind(t2),0,cosd(t2)];
    R = r_y*r_x*r_z;
    face_s_local(1:3,:) = R\face_s_local(1:3,:);
    face_s_local(4:6,:) = R\face_s_local(4:6,:);
    face_s_local(7:9,:) = R\face_s_local(7:9,:);
    %
    d = cat(3,cos(psi_s).*cos(phi_s),sin(psi_s).*cos(phi_s),sin(phi_s));
    d = permute(d,[3,1,2]);

    num_face = size(face_s_local,2);
    A = reshape(face_s_local(1:3,:),3,1,1,num_face);
    AB = reshape(face_s_local(4:6,:) - face_s_local(1:3,:),3,1,1,num_face);
    AC = reshape(face_s_local(7:9,:) - face_s_local(1:3,:),3,1,1,num_face);
    AO = -A;
    %
    D = my_det(AB,AC,-d);
    D1 = my_det(AO,AC,-d);
    D2 = my_det(AB,AO,-d);
    D3 = my_det(AB,AC,AO);
    %
    u = D1./D;
    v = D2./D;
    t = D3./D;

    % in_face = squeeze((v>0&v+u<1) & (u>0&u<1));

    % P = t.*d;
    % P = P(:,in_face);
%     no_in_face = reshape((v<0|v+u>1) | (u<0|u>1) | t<0 | t>r_sense,size(u,2:4));
    no_in_face = reshape((v<0|v+u>1) | (u<0|u>1) | t<0 | t>r_sense | t<r_min,size(u,2:4));


    t = reshape(t,size(t,2:4));
    t(no_in_face) = inf;
    range_s = min(t,[],3);
else
    range_s = zeros(size(psi_s)) + inf;
end
    function results = my_det(v1,v2,v3)

    results = v1(1,:,:,:).*v2(2,:,:,:).*v3(3,:,:,:) +...
        v1(2,:,:,:).*v2(3,:,:,:).*v3(1,:,:,:) +...
        v1(3,:,:,:).*v2(1,:,:,:).*v3(2,:,:,:) -...
        v1(3,:,:,:).*v2(2,:,:,:).*v3(1,:,:,:) -...
        v1(2,:,:,:).*v2(1,:,:,:).*v3(3,:,:,:) -...
        v1(1,:,:,:).*v2(3,:,:,:).*v3(2,:,:,:);


    end

end

