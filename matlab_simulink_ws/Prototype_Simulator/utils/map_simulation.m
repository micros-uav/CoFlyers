function map_simulation(map3d_faces, map3d_struct, model_stls, ...
    x_range, y_range, z_range, app)
%MAP_SIMULATION test the simulation of map
if nargin < 7
    app = [];
    fig = figure;
    axi = gca;
else
    fig =  app.UIFigure;
    axi =  app.UIAxes;
end


time_max = 1000;
sample_time_motion = 0.1;
dim_visua = 3;

for t = 0:sample_time_motion:time_max
    if ~isempty(app)
        if strcmp(app.StartsimulationButton.Text,'Start simulation')
            break
        end
    end

    [map3d_faces,map3d_struct] =...
        map_module_update_map3d(t,sample_time_motion,map3d_faces,map3d_struct);
    draw_environment(axi, map3d_faces, map3d_struct, model_stls, dim_visua);

    title(axi,num2str(t,"Time %.2f s"));
    
    drawnow;
end


end

