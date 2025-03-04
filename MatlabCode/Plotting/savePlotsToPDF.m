function savePlotsToPDF(pdfFileName, epsFilePrefix, Matlab, Unity, unity_keys, times, sd, thd, showFigures, showLegends, plotControlParams)

    %% Data Loading
    % Check if the data is structured correctly
    if ~exist("Unity", 'var') || ~exist("Matlab", 'var') || ~exist("unity_keys", 'var') || ~exist("times", 'var')
        disp('Invalid input data format. Expected an array with the elements "[Matlab, Unity, unity_keys, and times]".');
        disp('If you prefer you can load directly your file:');
        [fileName, filePath] = uigetfile({'*.h5';'*.mat'}, 'Select data file');
        if fileName == 0
            return; % User canceled the file selection
        end
        fullFileName = fullfile(filePath, fileName);
    
        shortfileName = split(fileName,".");
        shortfileName = string(shortfileName(end));
    
        if(strcmp(shortfileName,"h5"))
            % We load the hdf5 file
            [times, Matlab, Unity, unity_keys] = customHDF5_read(fullFileName);
            shortFileName = split(fileName,'.');
            shortFileName = string(shortFileName(1));
            % We save the content into a .mat file for faster readings
            save(strcat(shortFileName,".mat"),"times","Matlab","Unity","unity_keys",'-mat');
    
        else
            % If it is directly a .mat file, we just load the file
            load(fullFileName);
        end        
    end

    if ~exist("sd", 'var')
        sd = 1;
    end

    if ~exist("thd", 'var')
        thd = 0;
    end

    if ~exist("showFigures", 'var')
        showFigures = false;
    end

    if ~exist("showLegends", 'var')
        showLegends = true;
    end

    if ~exist("plotControlParams", 'var')
        plotControlParams = true;
    end

    disp("Data Loaded successfully");

    %% Pdf and plot creation
    pdfFileName = strcat(pdfFileName, '.pdf');
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
        position_struct = agent_dict("position");
        for t = 1:n_steps
            x_all(t, i) = position_struct(t).x;
            y_all(t, i) = position_struct(t).y;
            z_all(t, i) = position_struct(t).z;
        end
        dest_dict = Unity(string(destination_keys(i)));
        x_dest(i) = dest_dict("position").x;
        y_dest(i) = dest_dict("position").y;
        z_dest(i) = dest_dict("position").z;
    end


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

    %% Generate 3D Trajectory plot
    fig = figure;
    colors = lines(n_agents + n_defObjs + n_agents); % Use the same colormap for consistency

    hold on;
    for agent = 1:n_agents
        plot3(x_all(:, agent), z_all(:, agent), y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        plot3(x_all(1, agent), z_all(1, agent), y_all(1, agent), 'o', 'MarkerSize', 8, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' start']);  % Start marker
        plot3(x_all(end, agent), z_all(end, agent), y_all(end, agent), '*', 'MarkerSize', 10, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' end']);  % End marker
        plot3(x_dest(agent), z_dest(agent), y_dest(agent), '+', 'MarkerSize', 12, 'Color', colors(agent, :), 'LineWidth', 2, 'DisplayName', ['Agent ' num2str(agent) ' dest']);  % Destination marker
    end
    xlabel('X Axis');
    ylabel('Z Axis');
    zlabel('Y Axis');
    view(45, 50);
    grid on;
    if showLegends
        legend('show');
    end
    hold off;

    % Store the 3D plot in the PDF
    if ~showFigures
        set(fig, 'Visible', 'off');
    end


    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agent 3D Trajectories', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    exportgraphics(fig, pdfFileName, 'Append', false, 'ContentType', 'vector');
    epsFileName = sprintf('%s_Trajectories.eps', epsFilePrefix);
    print(fig, '-depsc', epsFileName);
    close(fig);
    disp(['EPS file saved as: ' epsFileName]);

    %% Generate 3D plot Trajectory + DefObject
    fig = figure;

    hold on;
    % 3D plot of the Agents
    for agent = 1:n_agents
        plot3(x_all(:, agent), z_all(:, agent), y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        plot3(x_all(1, agent), z_all(1, agent), y_all(1, agent), 'o', 'MarkerSize', 8, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' start']);  % Start marker
        plot3(x_all(end, agent), z_all(end, agent), y_all(end, agent), '*', 'MarkerSize', 10, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' end']);  % End marker
        plot3(x_dest(agent), z_dest(agent), y_dest(agent), '+', 'MarkerSize', 12, 'Color', colors(agent, :), 'LineWidth', 2, 'DisplayName', ['Agent ' num2str(agent) ' dest']);  % Destination marker
    end

    % 3D plot of the Deformable Objects
    for defObj = 1:n_defObjs
        plot3(x_def_all(:, defObj), z_def_all(:, defObj), y_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :), 'DisplayName', ['DefObj ' num2str(defObj)]);
        plot3(x_def_all(1, defObj), z_def_all(1, defObj), y_def_all(1, defObj), 'o', 'MarkerSize', 8, 'Color', colors(n_agents + defObj, :), 'DisplayName', ['DefObj ' num2str(defObj) ' start']);  % Start marker
        plot3(x_def_all(end, defObj), z_def_all(end, defObj), y_def_all(end, defObj), '*', 'MarkerSize', 10, 'Color', colors(n_agents + defObj, :), 'DisplayName', ['DefObj ' num2str(defObj) ' end']);  % End marker
    end

    xlabel('X Axis');
    ylabel('Z Axis');
    zlabel('Y Axis');
    view(45, 50);
    grid on;
    if showLegends
        legend('show');
    end
    hold off;

    % Store the 3D plot in the PDF
    if ~showFigures
        set(fig, 'Visible', 'off');
    end


    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agent + DefObj 3D Trajectories', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    exportgraphics(fig, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_AllTrajectories.eps', epsFilePrefix);
    print(fig, '-depsc', epsFileName);
    close(fig);
    disp(['EPS file saved as: ' epsFileName]);

    %% Generate 3D plot Matlab Trajectory
    if plotControlParams
        fig = figure;
        % +++++++++++++++++++++++ Position +++++++++++++++++++++++
        % Preallocate arrays to store x, y, z data for all agents
        x_matlab_all = zeros(n_steps, n_agents);
        y_matlab_all = zeros(n_steps, n_agents);
        z_matlab_all = zeros(n_steps, n_agents);
    
        % Load destination positions
        x_matlab_dest = zeros(n_steps, n_agents);
        y_matlab_dest = zeros(n_steps, n_agents);
        z_matlab_dest = zeros(n_steps, n_agents);
    
        % Load positions and destination positions for agents
        for i = 1:n_agents
            pose_dict = Matlab("positions");
            dest_dict = Matlab("destinations");
            for t = 1:n_steps
                x_matlab_all(t, i) = pose_dict(t).x(i);
                y_matlab_all(t, i) = pose_dict(t).y(i);
                z_matlab_all(t, i) = pose_dict(t).z(i);
                x_matlab_dest(t,i) = dest_dict(t).x(i);
                y_matlab_dest(t,i) = dest_dict(t).y(i);
                z_matlab_dest(t,i) = dest_dict(t).z(i);
            end
        end
        hold on;
        % 3D plot of the Agents
        for agent = 1:n_agents
            plot3(x_matlab_all(:, agent), z_matlab_all(:, agent), y_matlab_all(:, agent), '-', 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent)]);
            plot3(x_matlab_all(1, agent), z_matlab_all(1, agent), y_matlab_all(1, agent), 'o', 'MarkerSize', 8, 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent) ' start']);  % Start marker
            plot3(x_matlab_all(end, agent), z_matlab_all(end, agent), y_matlab_all(end, agent), '*', 'MarkerSize', 10, 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent) ' end']);  % End marker
            plot3(x_matlab_dest(:, agent), z_matlab_dest(:, agent), y_matlab_dest(:, agent), '+', 'MarkerSize', 12, 'Color', colors(n_agents + n_defObjs + agent, :), 'LineWidth', 2, 'DisplayName', ['Matlab Agent ' num2str(agent) ' dest']);  % Destination marker
        end
    
        xlabel('X Axis');
        ylabel('Z Axis');
        zlabel('Y Axis');
        view(45, 50);
        grid on;
        if showLegends
            legend('show');
        end
        hold off;
    
        % Store the 3D plot in the PDF
        if ~showFigures
            set(fig, 'Visible', 'off');
        end
    
    
        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Agent 3D Trajectories', ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');
    
        exportgraphics(fig, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_MatlabTrajectories.eps', epsFilePrefix);
        print(fig, '-depsc', epsFileName);
        close(fig);
        disp(['EPS file saved as: ' epsFileName]);
    end

    % %% Generate 3D plot Trajectory comparison
    % if plotControlParams
    %     fig = figure;
    % 
    %     hold on;
    %     % 3D plot of the Agents
    %     for agent = 1:n_agents
    %         plot3(x_all(:, agent), z_all(:, agent), y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    %         plot3(x_all(1, agent), z_all(1, agent), y_all(1, agent), 'o', 'MarkerSize', 8, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' start']);  % Start marker
    %         plot3(x_all(end, agent), z_all(end, agent), y_all(end, agent), '*', 'MarkerSize', 10, 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' end']);  % End marker
    %         plot3(x_dest(agent), z_dest(agent), y_dest(agent), '+', 'MarkerSize', 12, 'Color', colors(agent, :), 'LineWidth', 2, 'DisplayName', ['Agent ' num2str(agent) ' dest']);  % Destination marker
    %         plot3(x_matlab_all(:, agent), z_matlab_all(:, agent), y_matlab_all(:, agent), '-', 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent)]);
    %         plot3(x_matlab_all(1, agent), z_matlab_all(1, agent), y_matlab_all(1, agent), 'o', 'MarkerSize', 8, 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent) ' start']);  % Start marker
    %         plot3(x_matlab_all(end, agent), z_matlab_all(end, agent), y_matlab_all(end, agent), '*', 'MarkerSize', 10, 'Color', colors(n_agents + n_defObjs + agent, :), 'DisplayName', ['Matlab Agent ' num2str(agent) ' end']);  % End marker
    %         plot3(x_matlab_dest(:, agent), z_matlab_dest(:, agent), y_matlab_dest(:, agent), '+', 'MarkerSize', 12, 'Color', colors(n_agents + n_defObjs + agent, :), 'LineWidth', 2, 'DisplayName', ['Matlab Agent ' num2str(agent) ' dest']);  % Destination marker
    %     end
    % 
    %     xlabel('X Axis');
    %     ylabel('Z Axis');
    %     zlabel('Y Axis');
    %     view(45, 50);
    %     grid on;
    %     if showLegends
    %         legend('show');
    %     end
    %     hold off;
    % 
    %     % Store the 3D plot in the PDF
    %     if ~showFigures
    %         set(fig, 'Visible', 'off');
    %     end
    % 
    % 
    %     % Add a title annotation (to act as a section heading)
    %     annotation('textbox', [0.5 1 0 0], 'String', 'Agent 3D Trajectories Comparison', ...
    %         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');
    % 
    %     exportgraphics(fig, pdfFileName, 'Append', true, 'ContentType', 'vector');
    %     epsFileName = sprintf('%s_3DTrajectoriesComparison.eps', epsFilePrefix);
    %     print(fig, '-depsc', epsFileName);
    %     close(fig);
    %     disp(['EPS file saved as: ' epsFileName]);
    % end

    %% Generate 2D Trajectory subplots per agent
    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, x_all(:, agent), '-', 'Color', colors(agent, :));
        hold on
        plot(times, linspace(x_dest(agent),x_dest(agent),n_steps), '-', 'Color','red');
        xlabel('Time (s)');
        ylabel('X Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, y_all(:, agent), '-', 'Color', colors(agent, :));
        hold on
        plot(times, linspace(y_dest(agent),y_dest(agent),n_steps), '-', 'Color','red');
        xlabel('Time (s)');
        ylabel('Y Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, z_all(:, agent), '-', 'Color', colors(agent, :));
        hold on
        plot(times, linspace(z_dest(agent),z_dest(agent),n_steps), '-', 'Color','red');
        xlabel('Time (s)');
        ylabel('Z Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' 2D Trajectories', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_Trajectories.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %% Generate 2D subplots per Deformable Object
    for defObj = 1:n_defObjs
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, x_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('X Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, y_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('Y Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, z_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('Z Position');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'DefObj ' + string(defObj) + ' 2D Trajectories', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_DefObj%s_Trajectories.eps', epsFilePrefix, string(defObj));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %% Generate 2D Trajectory subplots per agent and Matlab
    if plotControlParams
        for agent = 1:n_agents
            fig2D = figure;
            subplot(3, 1, 1);
            plot(times, x_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', 'Unity pos');
            hold on
            % plot(times, linspace(x_dest(agent),x_dest(agent),n_steps), '-', 'Color','red','DisplayName', 'Unity Dest');
            plot(times, x_matlab_all(:, agent), '-', 'Color', colors(agent + n_agents + n_defObjs, :), 'DisplayName', 'Matlab pos');
            % plot(times, x_matlab_dest(:, agent), '-', 'Color', colors(agent + n_agents, :), 'DisplayName', 'Matlab Dest');
            xlabel('Time (s)');
            ylabel('X Position');
            grid on;
            axis padded;
            if showLegends
                legend("show");
            end
            % title(['Agent ' num2str(agent) ' - X vs Time']);
    
            subplot(3, 1, 2);
            plot(times, y_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', 'Unity pos');
            hold on
            % plot(times, linspace(y_dest(agent),y_dest(agent),n_steps), '-', 'Color','red','DisplayName', 'Unity Dest');
            plot(times, y_matlab_all(:, agent), '-', 'Color', colors(agent + n_agents + n_defObjs, :), 'DisplayName', 'Matlab pos');
            % plot(times, y_matlab_dest(:, agent), '-', 'Color', colors(agent + n_agents, :), 'DisplayName', 'Matlab Dest');
            xlabel('Time (s)');
            ylabel('Y Position');
            grid on;
            axis padded;
            if showLegends
                legend("show");
            end
            % title(['Agent ' num2str(agent) ' - Y vs Time']);
    
            subplot(3, 1, 3);
            plot(times, z_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', 'Unity pos');
            hold on
            % plot(times, linspace(z_dest(agent),z_dest(agent),n_steps), '-', 'Color','red','DisplayName', 'Unity Dest');
            plot(times, z_matlab_all(:, agent), '-', 'Color', colors(agent + n_agents + n_defObjs, :), 'DisplayName', 'Matlab pos');
            % plot(times, z_matlab_dest(:, agent), '-', 'Color', colors(agent + n_agents, :), 'DisplayName', 'Matlab Dest');
            xlabel('Time (s)');
            ylabel('Z Position');
            grid on;
            axis padded;
            if showLegends
                legend("show");
            end
            % title(['Agent ' num2str(agent) ' - Z vs Time']);
    
            if ~showFigures
            set(fig2D, 'Visible', 'off');
            end
    
            % Add a title annotation (to act as a section heading)
            annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' 2D Trajectories comparison with Matlab', ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');
    
            % Store the 2D subplots in the PDF
            exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
            epsFileName = sprintf('%s_Agent%s_MatlabCompTrajectories.eps', epsFilePrefix, string(agent));
            print(fig2D, '-depsc', epsFileName);
            disp(['EPS file saved as: ' epsFileName]);
    
            close(fig2D);
        end
    end

    %% Generate position error plots

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
        fig2D = figure;
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

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' 2D Position Error', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_PosError.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
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

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents 2D Position Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_PosError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % +++++++++++++++++++++++ Rotation +++++++++++++++++++++++
    %% Generate 2D Rotation subplots per agent

    % We generate the arrays that will store the rotation values
    roll_all = zeros(n_steps, n_agents); % Rotation in X
    yaw_all = zeros(n_steps, n_agents);  % Rotation in Y
    pitch_all = zeros(n_steps, n_agents);% Rotation in Z

    % Load destination rotations
    roll_dest = zeros(n_agents, 1);
    yaw_dest = zeros(n_agents, 1);
    pitch_dest = zeros(n_agents, 1);

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        rotation_struct = agent_dict("rotation");
        for t = 1:n_steps
            quatMatrix = [rotation_struct(t).w, rotation_struct(t).x, rotation_struct(t).y, rotation_struct(t).z];
            eulerAngles = quat2eul(quatMatrix, 'XZY');
            roll_all(t, i) = eulerAngles(1);
            yaw_all(t, i) = eulerAngles(2);
            pitch_all(t, i) = eulerAngles(3);
        end
        dest_dict = Unity(string(destination_keys(i)));
        quatMatrix = [dest_dict("rotation").w, dest_dict("rotation").x, dest_dict("rotation").y, dest_dict("rotation").z];
        eulerAngles = quat2eul(quatMatrix, 'XZY');
        roll_dest(i) = eulerAngles(1);
        yaw_dest(i) = eulerAngles(2);
        pitch_dest(i) = eulerAngles(3);
    end

    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, roll_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Roll (X-axis rotation)');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, yaw_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Yaw (Y-axis rotation)');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, pitch_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Pitch (Z-axis rotation)');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Rotation Values', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_Rotations.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %% Generate 2D subplots per Deformable Object
    % We generate the arrays that will store the rotation values
    roll_def_all = zeros(n_steps, n_agents); % Rotation in X
    yaw_def_all = zeros(n_steps, n_agents);  % Rotation in Y
    pitch_def_all = zeros(n_steps, n_agents);% Rotation in Z

    % Load positions and destination positions for agents
    for i = 1:n_agents
        defObj_dict = Unity(string(defIvj_keys(i)));
        rotation_struct = defObj_dict("rotation");
        for t = 1:n_steps
            quatMatrix = [rotation_struct(t).w, rotation_struct(t).x, rotation_struct(t).y, rotation_struct(t).z];
            eulerAngles = quat2eul(quatMatrix, 'XZY');
            roll_def_all(t, i) = eulerAngles(1);
            yaw_def_all(t, i) = eulerAngles(2);
            pitch_def_all(t, i) = eulerAngles(3);
        end
    end

    for defObj = 1:n_defObjs
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, roll_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('Roll (X-axis rotation)');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, yaw_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('Yaw (Y-axis rotation)');
        grid on;         axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, pitch_def_all(:, defObj), '-', 'Color', colors(n_agents + defObj, :));
        xlabel('Time (s)');
        ylabel('Pitch (Z-axis rotation)');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'DefObj ' + string(defObj) + ' Rotation Values', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_DefObj%s_Rotations.eps', epsFilePrefix, string(defObj));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %% Generate rotation error plots

    % Preallocate arrays to store x, y, z position errors data for all agents
    rot_Error_roll_all = zeros(n_steps, n_agents);
    rot_Error_yaw_all = zeros(n_steps, n_agents);
    rot_Error_pitch_all = zeros(n_steps, n_agents);

    for i = 1:n_agents
        rot_Error_roll_all(:,i) = roll_dest(i) - roll_all(:,i);
        rot_Error_yaw_all(:,i) = yaw_dest(i) - yaw_all(:,i);
        rot_Error_pitch_all(:,i) = pitch_dest(i) - pitch_all(:,i);
    end

    % Individual Plots
    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, rot_Error_roll_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('Roll Rotation Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, rot_Error_yaw_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('Yaw Rotation Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, rot_Error_pitch_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
        xlabel('Time (s)');
        ylabel('Pitch Rotation Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Rotation Error', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_RotError.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, rot_Error_roll_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Roll Rotation Error');
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
        plot(times, rot_Error_yaw_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Yaw Rotation Error');
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
        plot(times, rot_Error_pitch_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Pitch Rotation Error');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Rotation Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_RotError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % +++++++++++++++++++++++ Velocity +++++++++++++++++++++++
    %% Generate Velocity subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    velx_all = zeros(n_steps, n_agents);
    vely_all = zeros(n_steps, n_agents);
    velz_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        vel_struct = agent_dict("velocity");
        for t = 1:n_steps
            velx_all(t, i) = vel_struct(t).x;
            vely_all(t, i) = vel_struct(t).y;
            velz_all(t, i) = vel_struct(t).z;
        end
    end


    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, velx_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('X Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, vely_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Y Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, velz_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Z Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Velocities', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_Velocities.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, velx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Velocity');
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
        plot(times, vely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Velocity');
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
        plot(times, velz_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z Velocity');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Velocity', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_Velocity.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);


    % +++++++++++++++++++++++ Angular Velocity +++++++++++++++++++++++
    %% Generate Velocity subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    ang_velx_all = zeros(n_steps, n_agents);
    ang_vely_all = zeros(n_steps, n_agents);
    ang_velz_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        vel_struct = agent_dict("angular velocity");
        for t = 1:n_steps
            ang_velx_all(t, i) = vel_struct(t).x;
            ang_vely_all(t, i) = vel_struct(t).y;
            ang_velz_all(t, i) = vel_struct(t).z;
        end
    end


    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, ang_velx_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('X Angular Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, ang_vely_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Y Angular Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, ang_velz_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Z Angular Velocity');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Angular Velocities', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_AngVelocities.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, ang_velx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Angular Velocity');
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
        plot(times, ang_vely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Angular Velocity');
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
        plot(times, ang_velz_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z Angular Velocity');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Angular Velocity', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_AngVelocity.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % +++++++++++++++++++++++ Acceleration +++++++++++++++++++++++
    %% Generate Acceleration subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    accelx_all = zeros(n_steps, n_agents);
    accely_all = zeros(n_steps, n_agents);
    accelz_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        accel_struct = agent_dict("acceleration");
        for t = 1:n_steps
            accelx_all(t, i) = accel_struct(t).x;
            accely_all(t, i) = accel_struct(t).y;
            accelz_all(t, i) = accel_struct(t).z;
        end
    end


    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, accelx_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('X Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, accely_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Y Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, accelz_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Z Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Acceleration', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_Accel.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, accelx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Acceleration');
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
        plot(times, accely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Acceleration');
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
        plot(times, accelz_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z Acceleration');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Acceleration', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_Accel.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % ++++++++++++++++++++++++++++++++++++++ Accel comparison ++++++++++++++++++++++++++++++++++++++
    %% Generate Acceleration Error subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    Des_accelx_all = zeros(n_steps, n_agents);
    Des_accely_all = zeros(n_steps, n_agents);
    Des_accelz_all = zeros(n_steps, n_agents);


    % % Load positions and destination positions for agents
    for i = 1:n_agents
        accel_struct = Matlab("acceleration");
        for t = 1:n_steps
            Des_accelx_all(t, i) = accel_struct(t).x(i);
            Des_accely_all(t, i) = accel_struct(t).y(i);
            Des_accelz_all(t, i) = accel_struct(t).z(i);
        end
    end
    


    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, Des_accelx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        hold on;
        plot(times, Des_accelx_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
        hold off;
        xlabel('Time (s)');
        ylabel('X Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, accely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        hold on;
        plot(times, Des_accely_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
        hold off;
        xlabel('Time (s)');
        ylabel('Y Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, accelz_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        hold on;
        plot(times, Des_accelz_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
        hold off;
        xlabel('Time (s)');
        ylabel('Z Acceleration');
        grid on;         axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Acceleration Comparison', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_AccelComp.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, accelx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        plot(times, Des_accelx_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
    end
    xlabel('Time (s)');
    ylabel('X Acceleration');
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
        plot(times, accely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        plot(times, Des_accely_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
    end
    xlabel('Time (s)');
    ylabel('Y Acceleration');
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
        plot(times, accelz_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent) ' real accel']);
        plot(times, Des_accelz_all(:, agent), '-', 'Color', 'red', 'DisplayName', ['Agent ' num2str(agent) ' desired accel']);
    end
    xlabel('Time (s)');
    ylabel('Z Acceleration');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Acceleration Comparison', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_AccelComp.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % ++++++++++++++++++++++++++++++++++++++ Accel Error ++++++++++++++++++++++++++++++++++++++
    %% Generate Acceleration Error subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    Error_accelx_all = zeros(n_steps, n_agents);
    Error_accely_all = zeros(n_steps, n_agents);
    Error_accelz_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    % for i = 1:n_agents
    %     Error_accelx_all(:, i) = Des_accelx_all(:,i) - accelx_all(:,i);
    %     Error_accely_all(:, i) = Des_accely_all(:,i) - accely_all(:,i);
    %     Error_accelz_all(:, i) = Des_accelz_all(:,i) - accelz_all(:,i);
    % end
    for i = 1:n_agents
        for t=2:n_steps
            Error_accelx_all(t, i) = Des_accelx_all(t-1,i) - accelx_all(t,i);
            Error_accely_all(t, i) = Des_accely_all(t-1,i) - accely_all(t,i);
            Error_accelz_all(t, i) = Des_accelz_all(t-1,i) - accelz_all(t,i);
        end
    end



    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, Error_accelx_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('X Acceleration Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, Error_accely_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Y Acceleration Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, Error_accelz_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Z Acceleration Error');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Acceleration Error', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_AccelError.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, Error_accelx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Acceleration Error');
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
        plot(times, Error_accely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Acceleration Error');
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
        plot(times, Error_accelz_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)']);
    end
    xlabel('Time (s)');
    ylabel('Z Acceleration Error');
    grid on;
    axis padded;
    if showLegends
        legend("show");
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Acceleration Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_AccelError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);


    % +++++++++++++++++++++++ Angular Acceleration +++++++++++++++++++++++
    %% Generate Acceleration subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    ang_accelx_all = zeros(n_steps, n_agents);
    ang_accely_all = zeros(n_steps, n_agents);
    ang_accelz_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        accel_struct = agent_dict("angular acceleration");
        for t = 1:n_steps
            ang_accelx_all(t, i) = accel_struct(t).x;
            ang_accely_all(t, i) = accel_struct(t).y;
            ang_accelz_all(t, i) = accel_struct(t).z;
        end
    end


    for agent = 1:n_agents
        fig2D = figure;
        subplot(3, 1, 1);
        plot(times, ang_accelx_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('X Angular Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - X vs Time']);

        subplot(3, 1, 2);
        plot(times, ang_accely_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Y Angular Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Y vs Time']);

        subplot(3, 1, 3);
        plot(times, ang_accelz_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Z Angular Acceleration');
        grid on;
        axis padded;
        % title(['Agent ' num2str(agent) ' - Z vs Time']);

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Angular Acceleration', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_AngAccel.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    %%% Group plots
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, ang_accelx_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X Angular Acceleration');
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
        plot(times, ang_accely_all(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y Angular Acceleration');
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
        plot(times, ang_accelz_all(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z Angular Acceleration');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Agents Angular Acceleration', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_All_Agents_AngAccel.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % +++++++++++++++++++++++ Grabbed +++++++++++++++++++++++
    %% Generate Grabbed subplots per agent
    % Preallocate arrays to store x, y, z velocities for all agents
    grabbed_all = zeros(n_steps, n_agents);
    

    % Load positions and destination positions for agents
    for i = 1:n_agents
        agent_dict = Unity(string(agent_keys(i)));
        grabbed_all(:, i) = agent_dict("grabbed");
    end


    for agent = 1:n_agents
        fig2D = figure;
        plot(times, grabbed_all(:, agent), '-', 'Color', colors(agent, :));
        xlabel('Time (s)');
        ylabel('Grabbed');
        grid on;
        axis padded;

        if ~showFigures
        set(fig2D, 'Visible', 'off');
        end

        % Add a title annotation (to act as a section heading)
        annotation('textbox', [0.5 1 0 0], 'String', 'Agent ' + string(agent) + ' Grabbed', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

        % Store the 2D subplots in the PDF
        exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
        epsFileName = sprintf('%s_Agent%s_Grabbed.eps', epsFilePrefix, string(agent));
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);

        close(fig2D);
    end

    % +++++++++++++++++++++++ Errors +++++++++++++++++++++++
    %% Generate Error plots
    gamma = zeros(n_steps,1);
    eg = zeros(n_steps,1);
    es = zeros(n_steps,1);
    eth = zeros(n_steps,1);

    for t = 1:n_steps
        poses = [x_all(t,:)' z_all(t,:)'];
        [gamma(t), poseError, es(t), eth(t)] = compute_errors(poses',[x_dest z_dest]', sd, thd);
        eg(t) = norm(poseError);
    end


    % --- Deformation error (gamma) ---
    fig2D = figure;
    plot(times, gamma(:), '-', 'Color', colors(1, :));
    xlabel('Time (s)');
    ylabel('Deformation Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Deformation Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_DeformationError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Position error (eg) ---
    fig2D = figure;
    plot(times, eg(:), '-', 'Color', colors(2, :));
    xlabel('Time (s)');
    ylabel('Position Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Position Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_PositionError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Scale error (es) ---
    fig2D = figure;
    plot(times, es(:), '-', 'Color', colors(3, :));
    xlabel('Time (s)');
    ylabel('Scale Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Scale Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_ScaleError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Rotation error (eth) ---
    fig2D = figure;
    plot(times, eth(:), '-', 'Color', colors(4, :));
    xlabel('Time (s)');
    ylabel('Rotation Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Rotation Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_RotationError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- All Errors ---
    fig2D = figure;
    plot(times, gamma(:), '-', 'Color', colors(1, :), 'DisplayName', "gamma");
    hold on;
    plot(times, eg(:), '-', 'Color', colors(2, :), 'DisplayName', "eg");
    plot(times, es(:), '-', 'Color', colors(3, :), 'DisplayName', "es");
    plot(times, eth(:), '-', 'Color', colors(4, :), 'DisplayName', "eth");
    hold off;
    xlabel('Time (s)');
    ylabel('Control Errors');
    grid on;
    axis padded;

    if showLegends
        legend("show");
    end
    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Control Errors', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_ControlErrors.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    if ~plotControlParams
        disp(['Plots saved in A4 PDF: ' pdfFileName]);
        return;
    end

    %% Generate Matlab Error plots
    gamma_mat = Matlab("gamma");
    eg_mat = Matlab("gamma");
    es_mat = Matlab("gamma");
    eth_mat = Matlab("gamma");


    % --- Deformation error (gamma) ---
    fig2D = figure;
    plot(times, gamma_mat(:), '-', 'Color', colors(1, :));
    xlabel('Time (s)');
    ylabel('Deformation Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Deformation Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_MatlabDeformationError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Position error (eg) ---
    fig2D = figure;
    plot(times, eg_mat(:), '-', 'Color', colors(2, :));
    xlabel('Time (s)');
    ylabel('Position Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Position Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_MatlabPositionError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Scale error (es) ---
    fig2D = figure;
    plot(times, es_mat(:), '-', 'Color', colors(3, :));
    xlabel('Time (s)');
    ylabel('Scale Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Scale Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_MatlabScaleError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Rotation error (eth) ---
    fig2D = figure;
    plot(times, eth_mat(:), '-', 'Color', colors(4, :));
    xlabel('Time (s)');
    ylabel('Rotation Error');
    grid on;
    axis padded;

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Rotation Error', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_MatlabRotationError.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- All Errors ---
    fig2D = figure;
    plot(times, gamma_mat(:), '-', 'Color', colors(1, :), 'DisplayName', "gamma");
    hold on;
    plot(times, eg_mat(:), '-', 'Color', colors(2, :), 'DisplayName', "eg");
    plot(times, es_mat(:), '-', 'Color', colors(3, :), 'DisplayName', "es");
    plot(times, eth_mat(:), '-', 'Color', colors(4, :), 'DisplayName', "eth");
    hold off;
    xlabel('Time (s)');
    ylabel('Control Errors');
    grid on;
    axis padded;

    if showLegends
        legend("show");
    end
    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Matlab Control Errors', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_MatlabControlErrors.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    if ~plotControlParams
        disp(['Plots saved in A4 PDF: ' pdfFileName]);
        return;
    end

    %% Generate Error Comparison plots

    % --- Deformation error (gamma) ---
    fig2D = figure;
    hold on;
    plot(times, gamma(:),'-', 'Color', colors(1,:), 'DisplayName', 'Unity');
    plot(times, gamma_mat(:), '-', 'Color', 'red', 'DisplayName', 'Matlab');
    hold off;
    xlabel('Time (s)');
    ylabel('Deformation Error');
    grid on;
    axis padded;
    if showLegends
        legend("show");
    end

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Deformation Error Comparison', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_DeformationErrorComparison.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Position error (eg) ---
    fig2D = figure;
    hold on;
    plot(times, eg(:),'-', 'Color', colors(2,:), 'DisplayName', 'Unity');
    plot(times, eg_mat(:), '-', 'Color', 'red', 'DisplayName', 'Matlab');
    hold off;
    xlabel('Time (s)');
    ylabel('Position Error');
    grid on;
    axis padded;
    if showLegends
        legend("show");
    end

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Position Error Comparison', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_PositionErrorComparison.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Scale error (es) ---
    fig2D = figure;
    hold on;
    plot(times, es(:),'-', 'Color', colors(3,:), 'DisplayName', 'Unity');
    plot(times, es_mat(:), '-', 'Color', 'red', 'DisplayName', 'Matlab');
    hold off;
    xlabel('Time (s)');
    ylabel('Scale Error');
    grid on;
    axis padded;
    if showLegends
        legend("show");
    end

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Scale Error Comparison', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_ScaleErrorComparison.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    % --- Rotation error (eth) ---
    fig2D = figure;
    hold on;
    plot(times, eth(:),'-', 'Color', colors(4,:), 'DisplayName', 'Unity');
    plot(times, eth_mat(:), '-', 'Color', 'red', 'DisplayName', 'Matlab');
    hold off;
    xlabel('Time (s)');
    ylabel('Rotation Error');
    grid on;
    axis padded;
    if showLegends
        legend("show");
    end

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Rotation Error Comparison', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_RotationErrorComparison.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    %% ++++++++++ U_D ++++++++++
    %%% Group plots
    U_Dx = zeros(n_steps, n_agents);
    U_Dy = zeros(n_steps, n_agents);
    U_Dz = zeros(n_steps, n_agents);

    UD_dict = Matlab("U_D");
    for t = 1:n_steps
        U_Dx(t,:) = UD_dict(t).x;
        U_Dy(t,:) = UD_dict(t).y;
        U_Dz(t,:) = UD_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_Dx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Deformation Control Value');
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
        plot(times, U_Dy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Deformation Control Value');
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
        plot(times, U_Dz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Deformation Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Deformation Control Value (U_D)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_DeformationControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    %% ++++++++++ U_G ++++++++++
    %%% Group plots
    U_Gx = zeros(n_steps, n_agents);
    U_Gy = zeros(n_steps, n_agents);
    U_Gz = zeros(n_steps, n_agents);

    UG_dict = Matlab("U_G");
    for t = 1:n_steps
        U_Gx(t,:) = UG_dict(t).x;
        U_Gy(t,:) = UG_dict(t).y;
        U_Gz(t,:) = UG_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_Gx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Correction Term Deformation Control Value');
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
        plot(times, U_Gy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Correction Term Deformation Control Value');
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
        plot(times, U_Gz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Correction Term Deformation Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Correction Term Deformation Control Value (U_G)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_CorrectTermDeformationControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    %% ++++++++++ U_H ++++++++++
    %%% Group plots
    U_Hx = zeros(n_steps, n_agents);
    U_Hy = zeros(n_steps, n_agents);
    U_Hz = zeros(n_steps, n_agents);

    UH_dict = Matlab("U_H");
    for t = 1:n_steps
        U_Hx(t,:) = UH_dict(t).x;
        U_Hy(t,:) = UH_dict(t).y;
        U_Hz(t,:) = UH_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_Hx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Internal Deformation Control Value');
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
        plot(times, U_Hy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Internal Deformation Control Value');
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
        plot(times, U_Hz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Internal Deformation Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Internal Deformation Control Value (U_H)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_InternalDeformationControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    %% ++++++++++ U_gamma ++++++++++
    %%% Group plots
    U_gammax = zeros(n_steps, n_agents);
    U_gammay = zeros(n_steps, n_agents);
    U_gammaz = zeros(n_steps, n_agents);

    Ugamma_dict = Matlab("U_gamma");
    for t = 1:n_steps
        U_gammax(t,:) = Ugamma_dict(t).x;
        U_gammay(t,:) = Ugamma_dict(t).y;
        U_gammaz(t,:) = Ugamma_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_gammax(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Shape Deformation Control Value');
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
        plot(times, U_gammay(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Shape Deformation Control Value');
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
        plot(times, U_gammaz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Shape Deformation Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Shape Deformation Control Value (U_gamma)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_ShapeDeformationControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);


    %% ++++++++++ U_s ++++++++++
    %%% Group plots
    U_sx = zeros(n_steps, n_agents);
    U_sy = zeros(n_steps, n_agents);
    U_sz = zeros(n_steps, n_agents);

    Us_dict = Matlab("U_s");
    for t = 1:n_steps
        U_sx(t,:) = Us_dict(t).x;
        U_sy(t,:) = Us_dict(t).y;
        U_sz(t,:) = Us_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_sx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Scale Control Value');
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
        plot(times, U_sy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Scale Control Value');
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
        plot(times, U_sz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Scale Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Scale Control Value (U_s)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_ScaleControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);

    %% ++++++++++ U_g ++++++++++
    %%% Group plots
    U_gx = zeros(n_steps, n_agents);
    U_gy = zeros(n_steps, n_agents);
    U_gz = zeros(n_steps, n_agents);

    Ug_dict = Matlab("U_g");
    for t = 1:n_steps
        U_gx(t,:) = Ug_dict(t).x;
        U_gy(t,:) = Ug_dict(t).y;
        U_gz(t,:) = Ug_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_gx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Position Control Value');
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
        plot(times, U_gy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Position Control Value');
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
        plot(times, U_gz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Position Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Position Control Value (U_g)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_PositionControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);    

    %% ++++++++++ U_th ++++++++++
    %%% Group plots
    U_thx = zeros(n_steps, n_agents);
    U_thy = zeros(n_steps, n_agents);
    U_thz = zeros(n_steps, n_agents);

    Uth_dict = Matlab("U_th");
    for t = 1:n_steps
        U_thx(t,:) = Uth_dict(t).x;
        U_thy(t,:) = Uth_dict(t).y;
        U_thz(t,:) = Uth_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_thx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Rotation Control Value');
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
        plot(times, U_thy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Rotation Control Value');
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
        plot(times, U_thz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Rotation Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Rotation Control Value (U_th)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_RotationControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);    

    %% ++++++++++ U_f ++++++++++
    %%% Group plots
    U_fx = zeros(n_steps, n_agents);
    U_fy = zeros(n_steps, n_agents);
    U_fz = zeros(n_steps, n_agents);

    Uf_dict = Matlab("U_f");
    for t = 1:n_steps
        U_fx(t,:) = Uf_dict(t).x;
        U_fy(t,:) = Uf_dict(t).y;
        U_fz(t,:) = Uf_dict(t).z;
    end
    
    fig2D = figure;
    subplot(3, 1, 1);
    hold on;
    for agent = 1:n_agents
        plot(times, U_fx(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('X - Global Control Value');
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
        plot(times, U_fy(:, agent), '-', 'Color', colors(agent, :), 'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Y - Global Control Value');
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
        plot(times, U_fz(:, agent), '-', 'Color', colors(agent, :),'DisplayName', ['Agent ' num2str(agent)]);
    end
    xlabel('Time (s)');
    ylabel('Z - Global Control Value');
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;
    % title(['Agent ' num2str(agent) ' - Z vs Time']);

    if ~showFigures
    set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', 'Global Control Value (U_f)', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    exportgraphics(fig2D, pdfFileName, 'Append', true, 'ContentType', 'vector');
    epsFileName = sprintf('%s_GlobalControlVal.eps', epsFilePrefix);
    print(fig2D, '-depsc', epsFileName);
    disp(['EPS file saved as: ' epsFileName]);

    close(fig2D);   


    disp(['Plots saved in A4 PDF: ' pdfFileName]);
    % disp(['EPS file saved as: ' epsFileName]);
end
