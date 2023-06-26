function [myFigure,myAxes] = draw_monte_carlo_2D(fx,fy,ff,labels,w,h)
%PLOT_MODEL_DRAW_MONTE_CARLO_2D 


myFigure = figure;
myAxes   = axes;
box(myAxes,'on');

% x = 1:length(fx);
% y = 1:length(fy);
% [xx,yy] = meshgrid(x,y);
% 
% % w = 0.4;
% % h = 0.3;
% 
% x = xx(:)';
% x = [x+w;x+w;x-w;x-w];
% y = yy(:)';
% y = [y+h;y-h;y-h;y+h];
% 
% c = ff(:);
% 
% p1 = patch(x,y,c/max(c,[],'all'));
% p1.LineStyle = 'none';
% c = colorbar;
% xlabel(labels{1});
% ylabel(labels{2});
% c.Label.String = labels{3};
% set(gca,'FontName','Times New Roman','FontSize',12)
% xlim([1-2*w,length(fx)+2*w]);
% ylim([1-2*h,length(fy)+2*h])
% xticks(1:length(fx));
% yticks(1:length(fy));
% xticklabels(string(fx));
% yticklabels(string(fy));
heatmap(fx,flip(fy),flip(ff))
% c = colorbar;
xlabel(labels{1});
ylabel(labels{2});
% c.Label.String = labels{3};
set(gca,'FontName','Times New Roman','FontSize',12)
% xlim([1-2*w,length(fx)+2*w]);
% ylim([1-2*h,length(fy)+2*h])
% xticks(1:length(fx));
% yticks(1:length(fy));
% xticklabels(string(fx));
% yticklabels(string(fy));
end

