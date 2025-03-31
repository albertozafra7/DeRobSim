% Data is a struct that has to contain the following params:
% 1  - obstacles1: (3 x N_obstacles) array that contains the obstacle positions in the scenario
% 2  - obstacles2: (3 x N_obstacles) array that contains the obstacle positions in the scenario
% 3  - p0: (3*N_agents x 1) vector that contains the initial positions of the agents
% 4  - g0: (3 x 1) vector that contains the initial position of the agents' centroid
% 5  - PT: (3*N_agents x 1) vector that contains the final configuration of the agents with centroid in gd
% 6  - gd: (3 x 1) vector that contains the final position of the agents' centroid
% 7  - pairs: (N_pairs x 2) vector that contains the indices of nodes that are interconnected for a fair reconstruction of the object
% 8  - save_plots: Boolean that determines if the plots are going to be saved or not
% 9  - folder_result: String that specifies the parent folder where the plots are going to be saved
% 10 - niters: Scalar that specifies the number of iterations of the simulation
% 11 - dt: Float that specifies the timestep of each iteration
% 12 - gammas_H: (N_iters x 1) vector that specifies the cost relative to shape-preserving transformation
% 13 - gammas_G: (N_iters x 1) vector that species the cost relative to configuration consistent with our deformation modes
% 14 - egs: (N_iters x 1) vector that specifies the centroid position errors
% 15 - ess: (N_iters x 1) vector that specifies the scale error
% 16 - eths: (N_iters x 1) vector that specifies the global rotation error
% 17 - eth_xs: (N_iters x 1) vector that specifies the roll rotation error
% 18 - eth_ys: (N_iters x 1) vector that specifies the pitch rotation error
% 19 - eth_zs: (N_iters x 1) vector that specifies the yaw rotation error
% 20 - ps: (3*N_agents x N_iters) array containing the positions of the agents in each iteration
% 21 - vs: (3*N_agents x N_iters) array containing the velocities of the agents in each iteration of the simulation
% 22 - nvs: (N_agents x N_iters) array containing the velocity norm of the agents in each iteration of the simulation
% 23 - x_limit: (2 x 1) vector that determines the x limit of the plotting
% 24 - y_limit: (2 x 1) vector that determines the y limit of the plotting
% 25 - z_limit: (2 x 1) vector that determines the z limit of the plotting
% 26 - plotaxfont: String that contains the name of the font used for the plotting
% 27 - color_graphs: (1 x N_agents) string vector that contains the color representation the agents in the plots
% 28 - color_robots: (1 x N_errors) string vector that contains the color representation of the errors in the plotting
% 29 - view_1: Scalar which contains the angle of view of the 3D plots
% 30 - view_2: Scalar which contains the angle of view of the 3D plots
% 31 - plotTitleSufix: String that contains the sufix that has to be added to the title of the plot

function [] = plotControlResults(data)
% Plot robot paths, errors, positions and velocities.

N = size(data.p0,1)/3; % N_agents

%% Robot paths

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
grid on
hold on

view(data.view_1, data.view_2)

% Obtacles plotting
if exist('data.obstacles1', 'var')
    for i = 0:50
        plot3(data.obstacles1(1,:), data.obstacles1(2,:), data.obstacles1(3,:) + i*0.02, '-', 'color', "#b0aaa4", 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
if exist('data.obstacles2', 'var')
    for i = 0:10
        plot3(data.obstacles2(1,:), data.obstacles2(2,:), data.obstacles2(3,:) + i*0.02, '-', 'color', "#b0aaa4", 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
% Initial positions and centroid
plot3(data.p0(1:3:end), data.p0(2:3:end), data.p0(3:3:end), 'o', 'markersize', 8, 'color', [0.5 0.5 0.8], 'markerfacecolor', [0.5 0.5 0.8], 'DisplayName', 'Posiciones iniciales')
plot3(data.g0(1,:), data.g0(2,:), data.g0(3,:), '+', 'markersize', 5, 'color', [0.5 0.5 0.8], 'markerfacecolor', [0.5 0.5 0.8], 'linewidth', 1.2, 'HandleVisibility', 'off')

% Desired final positions and centroid
plot3(data.PT(1:3:end), data.PT(2:3:end), data.PT(3:3:end), 's', 'markersize', 10, 'color', [0.8 0.5 0.5], 'markerfacecolor', [0.8 0.5 0.5], 'DisplayName', 'Posiciones deseadas')
plot3(data.gd(1,:), data.gd(2,:), data.gd(3,:), '+', 'markersize', 5, 'color', [0.8 0.5 0.5], 'markerfacecolor', [0.8 0.5 0.5], 'linewidth', 1.2, 'HandleVisibility', 'off')

% Lines between robots
for i = 1:size(data.pairs,1)
    plot3([data.p0(3*data.pairs(i,1)-2), data.p0(3*data.pairs(i,2)-2)], [data.p0(3*data.pairs(i,1)-1), data.p0(3*data.pairs(i,2)-1)], [data.p0(3*data.pairs(i,1)), data.p0(3*data.pairs(i,2))], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1.5, 'HandleVisibility', 'off')
    plot3([data.PT(3*data.pairs(i,1)-2), data.PT(3*data.pairs(i,2)-2)], [data.PT(3*data.pairs(i,1)-1), data.PT(3*data.pairs(i,2)-1)], [data.PT(3*data.pairs(i,1)), data.PT(3*data.pairs(i,2))], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off')
end

% data.path of each robot
for i = 1:N
    path(i) = plot3(data.ps(3*i-2,:), data.ps(3*i-1,:), data.ps(3*i,:), '-', 'color', data.color_robots(i), 'linewidth', 2); % data.path lines
end

% Titles and dimensions
set(gca, 'FontSize', 12, 'FontName', data.plotaxfont); 
xlabel('x (m)', 'FontSize', 18)
ylabel('y (m)', 'FontSize', 18)
zlabel('z (m)', 'FontSize', 18)

title(strcat('Trayectorias de los robots',{' '},data.plotTitleSufix), 'FontSize', 14);
xlim(data.x_limit) % to define the bounds of the plot properly
ylim(data.y_limit)
zlim(data.z_limit)
daspect([1 1 1]) % to have same plotted size of units in x and y

% legend
for i = 1:N
    path(i).DisplayName = sprintf('Trayectoria Robot %i',i);
end
legend('FontSize', 10, 'Location', 'northeastoutside')

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','robot_paths'));
    print(strcat(data.folder_result,'/','robot_paths'),'-dpng','-r300')
end


%% Error variables

% Time axis
t_ax = linspace(0, data.niters*data.dt, data.niters); % time axis

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% Gammas
plot(t_ax, data.gammas_H, 'color', data.color_graphs(1), 'linewidth', 2)
plot(t_ax, data.gammas_G, '--', 'color', data.color_graphs(2), 'linewidth', 2)

% Centroid error
plot(t_ax, data.egs, 'color', data.color_graphs(3), 'linewidth', 2)

% Scale error
plot(t_ax, data.ess, 'color', data.color_graphs(4), 'linewidth', 2)

% Angle error
plot(t_ax, data.eths, 'color', data.color_graphs(5), 'linewidth', 2)

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Error variables', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Legend
legend('\gamma_{H}', '\gamma_{G}', '||e_g||', 'e_s', 'e_R', 'Location', 'NorthEast', 'FontSize', 14)

% Title
title(strcat('Error Variables',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','error_variables'));
    print(strcat(data.folder_result,'/','error_variables'),'-dpng','-r300')
end


%% Angles errors (x, y, z)

% We create the figure and set the parameters
figure
set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
box on
hold on

% X Angle error
plot(t_ax, data.eth_xs, 'color', data.color_graphs(1), 'linewidth', 2)

% Y Angle error
plot(t_ax, data.eth_ys, 'color', data.color_graphs(2), 'linewidth', 2)

% Z Angle error
plot(t_ax, data.eth_zs, 'color', data.color_graphs(3), 'linewidth', 2)

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Angle errors (rad)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% legend
legend('e_{th}_x', 'e_{th}_y', 'e_{th}_z', 'Location', 'NorthEast', 'FontSize', 14)

% Title
title(strcat(data.plotTitleSufix,{' '},'Angles Errors (x,y,z)'), 'FontSize', 14);


% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','angle_errors'));
    print(strcat(data.folder_result,'/','angle_errors'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.ps(3*i-2,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in X (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Position in X',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','position_X'));
    print(strcat(data.folder_result,'/','position_X'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.ps(3*i-1,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in Y (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Position in Y',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','position_Y'));
    print(strcat(data.folder_result,'/','position_Y'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.ps(3*i,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in Z (m)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% Legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Position in Z',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','position_Z'));
    print(strcat(data.folder_result,'/','position_Z'),'-dpng','-r300')
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
%     lines(i) = plot(t_ax, dx_unis(1:2:end,i), 'color', data.color_robots(i), 'linewidth', 2);
% end
% 
% % Titles and dimensions
% set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
% xlabel('Time (s)', 'FontSize', 18)
% ylabel('Unicycle linear velocity (m/s)', 'FontSize', 18)
% 
% xlim([0, t_ax(end)]);
% 
% % legend
% for i = 1:N
%     lines(i).DisplayName = sprintf('Robot %i',i);
% end
% legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
% 
% % Save plots
% if data.save_plots == 1
%     hgsave(strcat(data.folder_result,'/','unicycle_linear_vel'));
%     print(strcat(data.folder_result,'/','unicycle_linear_vel'),'-dpng','-r300')
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
%     lines(i) = plot(t_ax, dx_unis(2:2:end,i), 'color', data.color_robots(i), 'linewidth', 2);
% end
% 
% % Titles and dimensions
% set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
% xlabel('Time (s)', 'FontSize', 18)
% ylabel('Unicycle angular velocity (rad/s)', 'FontSize', 18)
% 
% xlim([0, t_ax(end)]);
% 
% % legend
% for i = 1:N
%     lines(i).DisplayName = sprintf('Robot %i',i);
% end
% legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
% 
% % Save plots
% if data.save_plots == 1
%     hgsave(strcat(data.folder_result,'/','unicycle_angular_vel'));
%     print(strcat(data.folder_result,'/','unicycle_angular_vel'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.vs(3*i-2,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in X (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Linear Velocity X',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','velocity_X'));
    print(strcat(data.folder_result,'/','velocity_X'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.vs(3*i-1,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in Y (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Linear Velocity Y',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','velocity_Y'));
    print(strcat(data.folder_result,'/','velocity_Y'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.vs(3*i,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity in Z (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Linear Velocity Z',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','velocity_Z'));
    print(strcat(data.folder_result,'/','velocity_Z'),'-dpng','-r300')
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
    lines(i) = plot(t_ax, data.nvs(i,:), 'color', data.color_robots(i), 'linewidth', 2);
end

% Titles and dimensions
set(gca, 'FontSize', 16, 'FontName', data.plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Linear velocity norm (m/s)', 'FontSize', 18)

xlim([0, t_ax(end)]);

% legend
for i = 1:N
    lines(i).DisplayName = sprintf('Robot %i',i);
end
legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')

% Title
title(strcat('Linear Velocity Norm',{' '},data.plotTitleSufix), 'FontSize', 14);

% Save plots
if data.save_plots == 1
    hgsave(strcat(data.folder_result,'/','velocity_norm'));
    print(strcat(data.folder_result,'/','velocity_norm'),'-dpng','-r300')
end

end