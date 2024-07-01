% Plot robot paths, positions, velocities, accelerations, and make a video
% of the paths of the robots and the transported object

% %% Robot paths
% figure(4)
% set(gcf, 'Position',  [300,100,800,450]);
% hold on
% for i=1:N
%     p_i(i) = plot(Ps(1,i), Ps(2,i), 'o', 'markersize', 7, 'color', [0.5 0.5 0.8], 'markerfacecolor', [0.5 0.5 0.8], 'HandleVisibility', 'off');
%     p_f(i) = plot(PT(1,i), PT(2,i), 's', 'markersize', 8, 'color', [0.8 0.5 0.5], 'markerfacecolor', [0.8 0.5 0.5], 'HandleVisibility', 'off');      
%     tr(i) = plot(Ps(1:2:end,i), Ps(2:2:end,i), '-', 'color', colour(i), 'linewidth', 1.5);        
%     if i>1
%         uistack(plot([PT(1,i-1) PT(1,i)], [PT(2,i-1) PT(2,i)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
%         uistack(plot([Ps(1,i-1) Ps(1,i)], [Ps(2,i-1) Ps(2,i)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
%     end
% %     if i==2 | i==3 | i==7 | i==8
% %         uistack(plot([PT(1,i-1) PT(1,i)], [PT(2,i-1) PT(2,i)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %         uistack(plot([Ps(1,i-1) Ps(1,i)], [Ps(2,i-1) Ps(2,i)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %     end
% %     if i==5 | i==6
% %         uistack(plot([PT(1,i-2) PT(1,i)], [PT(2,i-2) PT(2,i)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %         uistack(plot([Ps(1,i-2) Ps(1,i)], [Ps(2,i-2) Ps(2,i)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %     end
% %     if i==4 | i==8
% %         uistack(plot([PT(1,i-3) PT(1,i)], [PT(2,i-3) PT(2,i)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %         uistack(plot([Ps(1,i-3) Ps(1,i)], [Ps(2,i-3) Ps(2,i)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% %     end
% end
% if ob_type == 1
% uistack(plot([PT(1,1) PT(1,N)], [PT(2,1) PT(2,N)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% uistack(plot([Ps(1,1) Ps(1,N)], [Ps(2,1) Ps(2,N)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% end
% 
% % if ob_type == 1
% % uistack(plot([PT(1,1) PT(1,2)], [PT(2,1) PT(2,2)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% % uistack(plot([PT(1,5) PT(1,6)], [PT(2,5) PT(2,6)], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% % uistack(plot([Ps(1,1) Ps(1,2)], [Ps(2,1) Ps(2,2)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% % uistack(plot([Ps(1,5) Ps(1,6)], [Ps(2,5) Ps(2,6)], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1, 'HandleVisibility', 'off'), 'bottom');
% % end
% 
% box on
% title({'Trayectorias de los robots';' '}, 'FontSize', 12);
% xlim([-1.6 1.6]) % to define the bounds of the plot properly
% ylim([-1 1])
% daspect([1 1 1]) % to have same plotted size of units in x and y
% grid on
% set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
% xlabel('x (m)', 'FontSize', 18)
% ylabel('y (m)', 'FontSize', 18)
% 
% p_i(1).HandleVisibility = 'on';
% p_i(1).DisplayName = 'Posiciones iniciales';
% p_f(1).HandleVisibility = 'on';
% p_f(1).DisplayName = 'Posiciones finales';
% tr(1).DisplayName = ('Trayectoria Robot 1');
% tr(2).DisplayName = ('Trayectoria Robot 2');
% tr(3).DisplayName = ('Trayectoria Robot 3');
% tr(4).DisplayName = ('Trayectoria Robot 4');
% if i > 4
% tr(5).DisplayName = ('Trayectoria Robot 5');
% tr(6).DisplayName = ('Trayectoria Robot 6');
% tr(7).DisplayName = ('Trayectoria Robot 7');
% tr(8).DisplayName = ('Trayectoria Robot 8');
% end
% legend('FontSize', 10, 'Location', 'northeastoutside');
% 
% hgsave(strcat(folder_result, '/', 'robot_paths'));
% print(strcat(folder_result, '/', 'robot_paths'), '-depsc')
% print(strcat(folder_result, '/', 'robot_paths'), '-dpng', '-r300')
% 

%% Plot robot positions, velocities and accelerations

%Time axis
t_ax = linspace(0, nit*dt, nit);


figure(5)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, Ps(1:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont);
xlabel('Time (s)', 'FontSize', 18);
ylabel('Position in X (m)', 'FontSize', 18);
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_X_position'));
% print(strcat(folder_result,'/','robots_X_position'),'-depsc')
print(strcat(folder_result,'/','robots_X_position'),'-dpng','-r300')


figure(6)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, Ps(2:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Position in Y (m)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_Y_position'));
% print(strcat(folder_result,'/','robots_Y_position'),'-depsc')
print(strcat(folder_result,'/','robots_Y_position'),'-dpng','-r300')


figure(7)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, vs(1:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Velocity in X (m/s)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_X_velocity'));
% print(strcat(folder_result,'/','robots_X_velocity'),'-depsc')
print(strcat(folder_result,'/','robots_X_velocity'),'-dpng','-r300')


figure(8)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, vs(2:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Velocity in Y (m/s)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_Y_velocity'));
% print(strcat(folder_result,'/','robots_Y_velocity'),'-depsc')
print(strcat(folder_result,'/','robots_Y_velocity'),'-dpng','-r300')


figure(9)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, as(1:2:end,i), 'color', colour(i), 'linewidth',2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Accel. in X (m/s^2)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_X_acceleration'));
% print(strcat(folder_result,'/','robots_X_acceleration'),'-depsc')
print(strcat(folder_result,'/','robots_X_acceleration'),'-dpng','-r300')


figure(10)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, as(2:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12,'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Accel. in Y (m/s^2)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_Y_acceleration'));
% print(strcat(folder_result,'/','robots_Y_acceleration'),'-depsc')
print(strcat(folder_result,'/','robots_Y_acceleration'),'-dpng','-r300')


figure(11)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, nvs(:,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Velocity norm (m/s)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_velocity_norm'));
% print(strcat(folder_result,'/','robots_velocity_norm'),'-depsc')
print(strcat(folder_result,'/','robots_velocity_norm'),'-dpng','-r300')


figure(12)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, nas(:,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Accel. norm (m/s^2)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_acceleration_norm'));
% print(strcat(folder_result,'/','robots_acceleration_norm'),'-depsc')
print(strcat(folder_result,'/','robots_acceleration_norm'),'-dpng','-r300')


figure(13)
set(gcf, 'Position',  [300,100,700,450]);
plot(t_ax, gammas, 'linewidth', 2)
hold on
plot(t_ax, egs, 'Linewidth', 2)
plot(t_ax, ess, 'Linewidth', 2)
plot(t_ax, eths, 'Linewidth', 2)

set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Error variables', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

legend('\gamma', '||e_g||', 'e_s', 'e_\theta', 'Location', 'NorthEast', 'FontSize', 15)

% hgsave(strcat(folder_result,'/','error_variables'));
% print(strcat(folder_result,'/','error_variables'),'-depsc')
print(strcat(folder_result,'/','error_variables'),'-dpng','-r300')


figure(14)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, dx_unis(1:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Unicycle linear velocity (m/s)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_unicycle_linear_velocity'));
% print(strcat(folder_result,'/','robots_unicycle_linear_velocity'),'-depsc')
print(strcat(folder_result,'/','robots_unicycle_linear_velocity'),'-dpng','-r300')


figure(15)
set(gcf, 'Position',  [300,100,700,450]);
hold on;
for i = 1:N
    lines(i) = plot(t_ax, dx_unis(2:2:end,i), 'color', colour(i), 'linewidth', 2);
end
set(gca, 'FontSize', 12, 'FontName', plotaxfont); 
xlabel('Time (s)', 'FontSize', 18)
ylabel('Unicycle angular velocity (rad/s)', 'FontSize', 18)
box on
xlim([0, t_ax(end)]);

lines(1).DisplayName = 'Robot 1';
lines(2).DisplayName = 'Robot 2';
lines(3).DisplayName = 'Robot 3';
lines(4).DisplayName = 'Robot 4';
if i > 4
lines(5).DisplayName = ('Robot 5');
% lines(6).DisplayName = ('Robot 6');
% lines(7).DisplayName = ('Robot 7');
% lines(8).DisplayName = ('Robot 8');
end
% legend('FontSize', 10, 'Location', 'northeastoutside');

% hgsave(strcat(folder_result,'/','robots_unicycle_angular_velocity'));
% print(strcat(folder_result,'/','robots_unicycle_angular_velocity'),'-depsc')
print(strcat(folder_result,'/','robots_unicycle_angular_velocity'),'-dpng','-r300')


% if make_video == 1
%     create_video(folder_result, 'video_object_transport', 'robots_object_', floor(300), fps, 1, 2)
% end