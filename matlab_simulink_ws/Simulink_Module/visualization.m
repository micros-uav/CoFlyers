%%
dim = 2;

file_name = "covering_han";
[map3d_faces, map3d_struct, model_stls, params, position0, param_simulink] =...
    read_parameter_xml(strcat("../Prototype_Simulator/xml_config_files/parameters_",file_name,".xml"), [], false);
mat_file = matfile(strcat("data_save\",file_name,".mat"));
number = size(position0,2);
sim_out = mat_file.out;
time_series = sim_out.tout;
states_series = sim_out.states.Data;
states_series = reshape(states_series,size(states_series,1),17,number);
states_series = permute(states_series,[2,3,1]);
states_series = states_series(6:end,:,:);

sample_time_motion = mat_file.sample_time_base;
sample_time_control = mat_file.sample_time_control;
temp = floor(sample_time_control/sample_time_motion);
time_series = time_series(1:temp:end);
a = squeeze(states_series(1,6,:));
iter_start = find(a~=0,1);

states_series = states_series(:,:,iter_start:end);
time_series = time_series(iter_start:end) - time_series(iter_start);

commands_series = sim_out.commands.Data;
commands_series = reshape(sim_out.commands.Data,size(commands_series,1),6,number);

id_informed = [];

iter_start = find(commands_series(:,2,1)==7,1);
iter_end = iter_start + 300/0.1;  
time_series = time_series(iter_start:iter_end) - time_series(iter_start);
states_series = states_series(:,:,iter_start:iter_end);

center_series = squeeze(mean(states_series,2));


number_real = mat_file.number_real;
params = mat_file.params;
x_range = params.visual.x_range;
y_range = params.visual.y_range;
z_range = params.visual.z_range;
%% Video


d_iter = 1;

flag_save_video = false;

myFigure = figure;
% axis equal;hold on;box on;
hold on;box on; axis equal;
xlim(x_range)
ylim(y_range)
zlim(z_range)
% c = colorbar;
% caxis([0,ceil(time_series(end))])
% c.Label.String = "Time (s)";
set(gca,'FontName',"Times New Roman","FontSize",14)
set(gcf,"Position",[10,100,1000,500])

view([45,30])
if flag_save_video
    myVideo = VideoWriter(strcat("data_save\",file_name,".avi"));
    myVideo.FrameRate = 10;
    %         myVideo.Quality = 95;
    open(myVideo);
end
p1 = plot3(nan,nan,nan,'-k','LineWidth',2);
p2 = plot3(nan,nan,nan,'o','Color','b','MarkerSize',5,'MarkerFaceColor','b');
p3 = plot3(nan,nan,nan,'o','Color','r','MarkerSize',5,'MarkerFaceColor','r');

for iter = 2:d_iter:length(time_series)
    % cla
% 
    %========Drones=========%
    xs = squeeze(states_series(1,:,1:iter))'; xs(end,:) = nan;
    ys = squeeze(states_series(2,:,1:iter))'; ys(end,:) = nan;
    zs = squeeze(states_series(3,:,1:iter))'; zs(end,:) = nan;
    % cs = repmat(time_series(1:size(xs,1)),1,size(xs,2)); cs(end,:) = nan;
    % p = patch(xs,ys,zs,cs,'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    % p = patch(xs,ys,cs,'EdgeColor','interp','MarkerFaceColor','flat','LineWidth',2,'EdgeAlpha',0.6);
    
    % plot3(center_series(1,1:iter),center_series(2,1:iter),center_series(3,1:iter),'-k','LineWidth',2)
    % plot3(xs(end-1,1:5),ys(end-1,1:5),zs(end-1,1:5),'o','Color','b','MarkerSize',5,'MarkerFaceColor','b');
    % plot3(xs(end-1,6:10),ys(end-1,6:10),zs(end-1,6:10),'o','Color','r','MarkerSize',5,'MarkerFaceColor','r')
    set(p1,"XData",center_series(1,1:iter),"YData",center_series(2,1:iter),"ZData",center_series(3,1:iter));
    set(p2,"XData",xs(end-1,1:number_real),"YData",ys(end-1,1:number_real),"ZData",zs(end-1,1:number_real));
    set(p3,"XData",xs(end-1,number_real+1:end),"YData",ys(end-1,number_real+1:end),"ZData",zs(end-1,number_real+1:end));
    %========Map============%
    draw_environment(gca, map3d_faces, map3d_struct,model_stls, 3);

    drawnow
%     pause(0.01)
    title(['Time: ',num2str(time_series(iter),"%.1f"),' s'])

    if flag_save_video
        A = getframe(myFigure);
        writeVideo(myVideo,A);
    end
end

if flag_save_video
    close(myVideo);
end