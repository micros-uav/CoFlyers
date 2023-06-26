function draw_body(my_axes_1, states, dim, flag_motion_model,len_arm, colors)
%VISUAL_MODULE_DRAW_BODY Summary of this function goes here
% my_axes_1: target axes
% states: states of agents, [num_states, num_agents] 
% dim: dimention. 3 or 2.
% flag_motion_model: type of the motion model.
% len_arm: length of the arm of the quadcopter.

num = size(states,2);

if isempty(colors)
    colors = zeros(3,num);
end

% Change the data format to fit the input of the 'plot/plot3' function
switch flag_motion_model
    case 0 %point mass
        bodies = cat(3,states(1,:),states(2,:),states(3,:));
    case 1 %quadcopter
        xyzs = states(1:3,:);

        points4 = len_arm*0.7071*[[1;1;0],[1;-1;0],[-1;-1;0],[-1;1;0]];
        points4s = repmat(points4,1,1,num);
        
% Rotation, but pagemtimes is only supportted by more than 2020a version.
%         eulers = states(7:9,:);
%         rotationMatrixs = euler2RotationMatrix(eulers);
%         points4s = pagemtimes(rotationMatrixs,points4s);
        
        points4xs = reshape(points4s(1,:,:),4,num);
        points4ys = reshape(points4s(2,:,:),4,num);
        points4zs = reshape(points4s(3,:,:),4,num);

        points4xs = points4xs + repmat(xyzs(1,:),4,1);
        points4ys = points4ys + repmat(xyzs(2,:),4,1);
        points4zs = points4zs + repmat(xyzs(3,:),4,1);

        xs = [points4xs([1,3],:);nan*zeros(1,num);points4xs([2,4],:);nan*zeros(1,num)];xs = xs(:);
        ys = [points4ys([1,3],:);nan*zeros(1,num);points4ys([2,4],:);nan*zeros(1,num)];ys = ys(:);
        zs = [points4zs([1,3],:);nan*zeros(1,num);points4zs([2,4],:);nan*zeros(1,num)];zs = zs(:);

        bodies = cat(3,xs,ys,zs);
    case 2
        bodies = cat(3,states(1,:),states(2,:),states(3,:),states(4,:),states(5,:),states(6,:));
    otherwise
        bodies = cat(3,states(1,:),states(2,:),states(3,:));
end

% when the function is called for the first time, make
% flag_init=true and the function will init the bodies of my_axes_1. 
% Otherwise, make it 0 and the function will update the trajectories data
% of my_axes_1. 
obj = findobj(my_axes_1.Children,'Tag','Body');
flag_init = isempty(obj);

if flag_init
    %====Initialize and label the image object of bodies=====%
    switch flag_motion_model
        case 0 %point mass
            if dim==3
                p = scatter3(my_axes_1,bodies(:,:,1),bodies(:,:,2),bodies(:,:,3));
            elseif dim == 2
                p = scatter(my_axes_1,bodies(:,:,1),bodies(:,:,2));
            end
            set(p,'Marker','o','CData',colors','MarkerFaceColor','flat');
        case 1 %quadcopter
            if dim==3
                p = plot3(my_axes_1,bodies(:,:,1),bodies(:,:,2),bodies(:,:,3));
            elseif dim == 2
                p = plot(my_axes_1,bodies(:,:,1),bodies(:,:,2));
            end
            set(p,'LineStyle','-','LineWidth',2,'Color','k','Marker','.','MarkerSize',10,'MarkerEdgeColor','r');
        case 2
            if dim==3
                p = quiver3(my_axes_1,bodies(:,:,1),bodies(:,:,2),bodies(:,:,3),bodies(:,:,4),bodies(:,:,5),bodies(:,:,6));
            elseif dim == 2
                p = quiver(my_axes_1,bodies(:,:,1),bodies(:,:,2),bodies(:,:,4),bodies(:,:,5));
            end
            set(p,'Marker','o','MarkerFaceColor',get(p,'Color'));
        otherwise
            if dim==3
                p = plot3(my_axes_1,bodies(:,:,1),bodies(:,:,2),bodies(:,:,3));
            elseif dim == 2
                p = plot(my_axes_1,bodies(:,:,1),bodies(:,:,2));
            end
            set(p,'LineStyle','none','Marker','o','MarkerFaceColor',get(p,'Color'));
    end
    p.Tag = 'Body';
else
    %====Update data====%
    
    switch flag_motion_model
        case 0 %point mass
            if dim==3
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2),'ZData',bodies(:,:,3));
            elseif dim ==2
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2));
            end
            set(obj,'CData',colors');
        case 1 %quadcopter
            if dim==3
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2),'ZData',bodies(:,:,3));
            elseif dim ==2
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2));
            end
        case 2
            if dim==3
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2),'ZData',bodies(:,:,3),...
                    'UData',bodies(:,:,4),'VData',bodies(:,:,5),'WData',bodies(:,:,6));
            elseif dim ==2
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2),...
                    'UData',bodies(:,:,4),'VData',bodies(:,:,5));
            end
        otherwise
            if dim==3
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2),'ZData',bodies(:,:,3));
            elseif dim ==2
                set(obj,'XData',bodies(:,:,1),'YData',bodies(:,:,2));
            end
    end
end


end

