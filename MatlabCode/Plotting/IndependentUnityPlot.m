%% Variable selection
var_name = "position";
showLegends = false;

%% Pdf and plot creation
n_steps = length(times);
n_agents = length(unity_keys.Agents); % Number of agents
n_defObjs = length(unity_keys.DefObj); % Number of deformable objecst
agent_keys = keys(unity_keys.Agents); % Keys for accessing the agents
destination_keys = keys(unity_keys.GameObj); % Keys for accessing the agent destinations
defObj_keys = keys(unity_keys.DefObj); % Keys for accessing the Deformable Objects


% +++++++++++++++++++++++ Position +++++++++++++++++++++++
% Preallocate arrays to store x, y, z data for all agents
x_all = zeros(n_steps, n_agents);
y_all = zeros(n_steps, n_agents);
z_all = zeros(n_steps, n_agents);

% Load destination positions
x_dest = zeros(n_agents, 1);
y_dest = zeros(n_agents, 1);
z_dest = zeros(n_agents, 1);

% Load positions and destination positions for agents
for i = 1:n_agents
    agent_dict = Unity(string(agent_keys(i)));
    position_struct = agent_dict(var_name);
    for t = 1:n_steps
        x_all(t, i) = position_struct(t).x;
        y_all(t, i) = position_struct(t).y;
        z_all(t, i) = position_struct(t).z;
    end
    if(strcmp(var_name,"position") || strcmp(var_name,"rotation"))
        dest_dict = Unity(string(destination_keys(i)));
        x_dest(i) = dest_dict(var_name).x;
        y_dest(i) = dest_dict(var_name).y;
        z_dest(i) = dest_dict(var_name).z;
    end
end

if(strcmp(var_name,"postion") || strcmp(var_name,"rotation"))

    % Preallocate arrays to store x, y, z data for all the deformable
    % objects
    x_def_all = zeros(n_steps, n_defObjs);
    y_def_all = zeros(n_steps, n_defObjs);
    z_def_all = zeros(n_steps, n_defObjs);
    
    % Load positions for deformable objects
    for i = 1:n_defObjs
        defObj_dict = Unity(string(defObj_keys(i)));
        position_struct = defObj_dict("position");
        for t = 1:n_steps
            x_def_all(t, i) = position_struct(t).x;
            y_def_all(t, i) = position_struct(t).y;
            z_def_all(t, i) = position_struct(t).z;
        end
    end
end

%% Plotting
agent = 1;

colors = lines(n_agents + n_defObjs); % Use the same colormap for consistency
figure;
plot(times, x_all(:, agent), '-', 'Color', colors(agent, :));
if(strcmp(var_name,"position") || strcmp(var_name,"rotation"))
        hold on;
        plot(times, linspace(x_dest(agent),x_dest(agent),n_steps), '-', 'Color','red');
end
xlabel('Time (s)');
ylabel(['X ' var_name]);
axis padded;
grid on;
title(['Agent ' num2str(agent) ' - X vs Time']);

figure;
plot(times, y_all(:, agent), '-', 'Color', colors(agent, :));
if(strcmp(var_name,"position") || strcmp(var_name,"rotation"))
        hold on;
        plot(times, linspace(y_dest(agent),y_dest(agent),n_steps), '-', 'Color','red');
end
xlabel('Time (s)');
ylabel(['Y ' var_name]);
axis padded;
grid on;
title(['Agent ' num2str(agent) ' - Y vs Time']);

figure;
plot(times, z_all(:, agent)', '-', 'Color', colors(agent, :));
if(strcmp(var_name,"position") || strcmp(var_name,"rotation"))
        hold on;
        plot(times, linspace(z_dest(agent),z_dest(agent),n_steps), '-', 'Color','red');
end
xlabel('Time (s)');
ylabel(['Z ' var_name]);
axis padded;
grid on;

%% Generate position error plots

if(strcmp(var_name,"position"))
    % Preallocate arrays to store x, y, z position errors data for all agents
    pose_Error_x_all = zeros(n_steps, n_agents);
    pose_Error_y_all = zeros(n_steps, n_agents);
    pose_Error_z_all = zeros(n_steps, n_agents);
    
    for i = 1:n_agents
        pose_Error_x_all(:,i) = x_dest(i) - x_all(:,i);
        pose_Error_y_all(:,i) = y_dest(i) - y_all(:,i);
        pose_Error_z_all(:,i) = z_dest(i) - z_all(:,i);
    end
    
    % Individual Plots
    for agent = 1:n_agents
        figure;
        subplot(3, 1, 1);
        plot(times, pose_Error_x_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('X Position Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);
    
        subplot(3, 1, 2);
        plot(times, pose_Error_y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('Y Position Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);
    
        subplot(3, 1, 3);
        plot(times, pose_Error_z_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('Z Position Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);
    end
    
    %%% Group plots
    figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, pose_Error_x_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Position Error');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - X vs Time']);
    
    subplot(3, 1, 2);
    hold on;
    for agent = 1:n_agents
        plot(times, pose_Error_y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Position Error');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Y vs Time']);
    
    subplot(3, 1, 3);
    hold on;
    for agent = 1:n_agents
        plot(times, pose_Error_z_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z Position Error');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);
end