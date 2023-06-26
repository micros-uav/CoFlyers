function [posS, velS] = map_module_generate_shill_agent(map3d_faces, map3d_struct, dr, dim, height)
    %==============deleta some faces in range=================%
    ind_range = find(map3d_struct(15,:) == -2);
    if ~isempty(ind_range)
        ind_1_range = map3d_struct(end-1,ind_range);
        ind_2_range = map3d_struct(end,ind_range);
    %     ind = arrayfun(@(x)ind_1_range(x):ind_2_range(x),1:length(ind_2_range));
        ind = [];
        for i = 1:length(ind_1_range)
            ind = [ind, ind_1_range(i):ind_2_range(i)];
        end
        x_range = [min(map3d_faces(1,ind)),max(map3d_faces(1,ind))];
        y_range = [min(map3d_faces(2,ind)),max(map3d_faces(2,ind))];
        z_range = [min(map3d_faces(3,ind)),max(map3d_faces(3,ind))];
        ind_delete = [];
        for i = 1:length(ind_1_range)
            ind_1 = ind_1_range(i);
            ind_2 = ind_2_range(i);
            for j = ind_1:ind_2
                ind_now = j;
                f_now = map3d_faces(:, ind_now);
                if abs(f_now(1) - f_now(4)) < 1e-8 && abs(f_now(1) - f_now(7)) < 1e-8
                    % x range
                    if abs(f_now(1) - x_range(1))<1e-8 || abs(f_now(1) - x_range(2))<1e-8
                        ind_delete = [ind_delete,ind_now];
                    elseif min([norm(f_now(1:3)-f_now(4:6)), norm(f_now(1:3)-f_now(7:9)), norm(f_now(4:6)-f_now(7:9))]) <...
                            max([y_range(2)-y_range(1),z_range(2)-z_range(1)])*0.02
                        ind_delete = [ind_delete,ind_now];
                    end
                elseif abs(f_now(2) - f_now(5)) < 1e-8 && abs(f_now(2) - f_now(8)) < 1e-8
                    % y range
                    if abs(f_now(2) - y_range(1))<1e-8 || abs(f_now(2) - y_range(2))<1e-8
                        ind_delete = [ind_delete,ind_now];
                    elseif min([norm(f_now(1:3)-f_now(4:6)), norm(f_now(1:3)-f_now(7:9)), norm(f_now(4:6)-f_now(7:9))]) <...
                            max([x_range(2)-x_range(1),z_range(2)-z_range(1)])*0.02
                        ind_delete = [ind_delete,ind_now];
                    end
                elseif abs(f_now(3) - f_now(6)) < 1e-8 && abs(f_now(3) - f_now(9)) < 1e-8
                    % z range
                    if abs(f_now(3) - z_range(1))<1e-8 || abs(f_now(3) - z_range(2))<1e-8
                        ind_delete = [ind_delete,ind_now];
                    elseif min([norm(f_now(1:3)-f_now(4:6)), norm(f_now(1:3)-f_now(7:9)), norm(f_now(4:6)-f_now(7:9))]) <...
                            max([x_range(2)-x_range(1),y_range(2)-y_range(1)])*0.02
                        ind_delete = [ind_delete,ind_now];
                    end
    
                end
            end
        end
        map3d_faces(:,ind_delete) = [];
    end
    %===============================================%

    if dim == 2
        A = [10000;0;height]; B = [0;10000;height]; C = [10000;10000;height];
        map3d_lines = get_map3d_profile(A, B, C, map3d_faces);
        map_lines = map3d_lines([1,2,4,5,7,8],:);
        [numS, posS, velS] = generateShills_2d(map_lines, dr);
    elseif dim == 3

        [numS, posS, velS] = generateShills_3d(map3d_faces, dr);
    end
    
    ind_delete = [];
    for i = 1:size(posS,2)
        posij = posS(:,i) - posS;
        posij_norm = vecnorm(posij,2,1);
        posij_norm(i) = inf;
        angle_ij = acos(sum(velS(:,i).*velS,1)/norm(velS(:,i))./vecnorm(velS,2,1));
        ind = find(posij_norm < dr*0.5 & abs(angle_ij) < pi/6); 
        ind_delete = unique([ind_delete, ind]);
        posS(:,ind) = 0;
    end
    posS(:,ind_delete) = [];
    velS(:,ind_delete) = [];
    numS = numS - length(ind_delete);
end

function [numS, posS, velS] = generateShills_2d(map_lines, dr)
    posS = [];
    velS = [];
    numS = 0;
    for m = 1:size(map_lines,2)
        ms = map_lines(1:2,m);
        me = map_lines(3:4,m);
        dis = norm(me - ms);
        numS_one = ceil(dis/dr)+1;
        posS_m = linspacePoints(ms,me,numS_one);
        posS = [posS,posS_m];
        vv = map_lines(5:6,m);
        velS = [velS,repmat(vv,1,numS_one)];
        numS = numS + numS_one;
    end
end

function [numS, posS, velS] = generateShills_3d(map_faces, dr)
    posS = [];
    velS = [];
    numS = 0;
    for m = 1:size(map_faces,2)
        A = map_faces(1:3,m);
        B = map_faces(4:6,m);
        C = map_faces(7:9,m);
        e1 = B-A;
        e2 = C-A;
        e1_norm = norm(e1);
        e2_norm = norm(e2);
        e1 = e1/e1_norm;
        e2 = e2/e2_norm;
        
        [xx,yy] = meshgrid([0:dr:e1_norm,e1_norm],[0:dr:e2_norm,e2_norm]);
        p = xx(:)'.*e1 + yy(:)'.*e2 + A;
        in_s = false(1,size(p,2));
        for i = 1:size(p,2)
            in = point_in_triangle(p(:,i),A,B,C);
            in_s(i) = in;
        end
        p(:,~in_s) = [];
        numS_one = size(p,2);
        posS = [posS,p];
        vv = map_faces(10:12,m);
        velS = [velS,repmat(vv,1,numS_one)];
        numS = numS + numS_one;
    end
end


function points = linspacePoints(point1,point2,num)
points = zeros(length(point1),num);
for ppp = 1:length(point1)
    points(ppp,:) = linspace(point1(ppp),point2(ppp),num);
end
end

