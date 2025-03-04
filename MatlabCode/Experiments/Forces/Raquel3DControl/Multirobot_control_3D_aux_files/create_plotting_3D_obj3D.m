% Plot robot paths, errors, positions and velocities.

%% Robot paths

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
grid on
hold on

view(view_1, view_2)

% Obtacles plotting
for i = 0:50
    plot3(obstacles1(1,:), obstacles1(2,:), obstacles1(3,:) + i*0.02, '-', 'color', "#b0aaa4", 'LineWidth', 2, 'HandleVisibility', 'off');
end
for i = 0:10
    plot3(obstacles2(1,:), obstacles2(2,:), obstacles2(3,:) + i*0.02, '-', 'color', "#b0aaa4", 'LineWidth', 2, 'HandleVisibility', 'off');
end

% Initial positions and centroid
plot3(p0(1:3:end), p0(2:3:end), p0(3:3:end), 'o', 'markersize', 8, 'color', [0.5 0.5 0.8], 'markerfacecolor', [0.5 0.5 0.8], 'DisplayName', 'Posiciones iniciales')
plot3(g0(1,:), g0(2,:), g0(3,:), '+', 'markersize', 5, 'color', [0.5 0.5 0.8], 'markerfacecolor', [0.5 0.5 0.8], 'linewidth', 1.2, 'HandleVisibility', 'off')

% Desired final positions and centroid
plot3(PT(1:3:end), PT(2:3:end), PT(3:3:end), 's', 'markersize', 10, 'color', [0.8 0.5 0.5], 'markerfacecolor', [0.8 0.5 0.5], 'DisplayName', 'Posiciones deseadas')
plot3(gd(1,:), gd(2,:), gd(3,:), '+', 'markersize', 5, 'color', [0.8 0.5 0.5], 'markerfacecolor', [0.8 0.5 0.5], 'linewidth', 1.2, 'HandleVisibility', 'off')

% Lines between robots
for i = 1:size(pairs,1)
    plot3([p0(3*pairs(i,1)-2), p0(3*pairs(i,2)-2)], [p0(3*pairs(i,1)-1), p0(3*pairs(i,2)-1)], [p0(3*pairs(i,1)), p0(3*pairs(i,2))], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1.5, 'HandleVisibility', 'off')
    plot3([PT(3*pairs(i,1)-2), PT(3*pairs(i,2)-2)], [PT(3*pairs(i,1)-1), PT(3*pairs(i,2)-1)], [PT(3*pairs(i,1)), PT(3*pairs(i,2))], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off')
end

% Path of each robot
for i = 1:N
    path(i) = plot3(ps(3*i-2,:), ps(3*i-1,:), ps(3*i,:), '-', 'color', color_robots(i), 'linewidth', 2); % path lines
end

% Titles and dimensions
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('x (m)', 'FontSize', 18)
ylabel('y (m)', 'FontSize', 18)
zlabel('z (m)', 'FontSize', 18)

title({'Trayectorias de los robots';' '}, 'FontSize', 14);
xlim(x_limit) % to define the bounds of the plot properly
ylim(y_limit)
zlim(z_limit)
daspect([1 1 1]) % to have same plotted size of units in x and y

% Leyend
for i = 1:N
    path(i).DisplayName = sprintf('Trayectoria Robot %i',i);
end
legend('FontSize', 10, 'Location', 'northeastoutside')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','robot_paths'));
    print(strcat(folder_result,'/','robot_paths'),'-dpng','-r300')
end


%% Error variables

% Time axis
t_ax = linspace(0, niters*dt, niters); % time axis

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Gammas
plot(t_ax, gammas_H, 'color', color_graphs(1), 'linewidth', 2)
plot(t_ax, gammas_G, '--', 'color', color_graphs(2), 'linewidth', 2)

% Centroid error
plot(t_ax, egs, 'color', color_graphs(3), 'linewidth', 2)

% Scale error
plot(t_ax, ess, 'color', color_graphs(4), 'linewidth', 2)

% Angle error
plot(t_ax, eths, 'color', color_graphs(5), 'linewidth', 2)

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Error variables', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
legend('\gamma_{H}', '\gamma_{G}', '||e_g||', 'e_s', 'e_R', 'Location', 'NorthEast', 'FontSize', 14)

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','error_variables'));
    print(strcat(folder_result,'/','error_variables'),'-dpng','-r300')
end


%% Angles errors (x, y, z)

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% X Angle error
plot(t_ax, eth_xs, 'color', color_graphs(1), 'linewidth', 2)

% Y Angle error
plot(t_ax, eth_ys, 'color', color_graphs(2), 'linewidth', 2)

% Z Angle error
plot(t_ax, eth_zs, 'color', color_graphs(3), 'linewidth', 2)

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Angle errors (rad)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
legend('e_{th}_x', 'e_{th}_y', 'e_{th}_z', 'Location', 'NorthEast', 'FontSize', 14)

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','angle_errors'));
    print(strcat(folder_result,'/','angle_errors'),'-dpng','-r300')
end


%% Positions

% POSITION X
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, ps(3*i-2,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in X (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','position_X'));
    print(strcat(folder_result,'/','position_X'),'-dpng','-r300')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% POSITION Y
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, ps(3*i-1,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in Y (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','position_Y'));
    print(strcat(folder_result,'/','position_Y'),'-dpng','-r300')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% POSITION Z
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, ps(3*i,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in Z (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','position_Z'));
    print(strcat(folder_result,'/','position_Z'),'-dpng','-r300')
end


%% Velocities

% % UNICYCLE LINEAR VELOCITY
% % We create the figure and set the parameters
% figure
% set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
% box on
% hold on
% 
% % Representation
% for i = 1:N
%     lines(i) = plot(t_ax, dx_unis(1:2:end,i), 'color', color_robots(i), 'linewidth', 2);
% end
% 
% % Titles and dimensions
% set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
% xlabel('Time (s)', 'FontSize', 18)
% ylabel('Unicycle linear velocity (m/s)', 'FontSize', 18)
% 
% xlim([0, t_ax(end)]);
% 
% % Leyend
% for i = 1:N
%     lines(i).DisplayName = sprintf('Robot %i',i);
% end
% legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
% 
% % Save plots
% if save_plots == 1
%     hgsave(strcat(folder_result,'/','unicycle_linear_vel'));
%     print(strcat(folder_result,'/','unicycle_linear_vel'),'-dpng','-r300')
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % UNICYCLE ANGULAR VELOCITY
% % We create the figure and set the parameters
% figure
% set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
% box on
% hold on
% 
% % Representation
% for i = 1:N
%     lines(i) = plot(t_ax, dx_unis(2:2:end,i), 'color', color_robots(i), 'linewidth', 2);
% end
% 
% % Titles and dimensions
% set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
% xlabel('Time (s)', 'FontSize', 18)
% ylabel('Unicycle angular velocity (rad/s)', 'FontSize', 18)
% 
% xlim([0, t_ax(end)]);
% 
% % Leyend
% for i = 1:N
%     lines(i).DisplayName = sprintf('Robot %i',i);
% end
% legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
% 
% % Save plots
% if save_plots == 1
%     hgsave(strcat(folder_result,'/','unicycle_angular_vel'));
%     print(strcat(folder_result,'/','unicycle_angular_vel'),'-dpng','-r300')
% end
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LINEAR VELOCITY IN X
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, vs(3*i-2,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in X (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','velocity_X'));
    print(strcat(folder_result,'/','velocity_X'),'-dpng','-r300')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LINEAR VELOCITY IN Y
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, vs(3*i-1,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in Y (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','velocity_Y'));
    print(strcat(folder_result,'/','velocity_Y'),'-dpng','-r300')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LINEAR VELOCITY IN Z
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, vs(3*i,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in Z (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','velocity_Z'));
    print(strcat(folder_result,'/','velocity_Z'),'-dpng','-r300')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LINEAR VELOCITY NORM
% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Representation
for i = 1:N
    lines(i) = plot(t_ax, nvs(i,:), 'color', color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity norm (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Leyend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Save plots
if save_plots == 1
    hgsave(strcat(folder_result,'/','velocity_norm'));
    print(strcat(folder_result,'/','velocity_norm'),'-dpng','-r300')
end
