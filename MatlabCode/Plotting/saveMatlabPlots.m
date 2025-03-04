function saveMatlabPlots(pdfFileName, epsFilePrefix, times, positions, destinations, accels, vels, UD, UG, UH, Ugamma, Us, Ug, Uth, Uf, gamma_err, eg_err, es_err, eth_err, saveEPS, showFigures, showLegends)

    if ~exist("saveEps", 'var')
        saveEPS = false;
    end

    if ~exist("showFigures", 'var')
        showFigures = false;
    end

    if ~exist("showLegends", 'var')
        showLegends = true;
    end


    %% Data proccessing
    n_steps = length(times);
    n_agents = size(positions.x); % Number of agents
    n_agents = n_agents(2);
    colors = lines(n_agents); % Use the same colormap for consistency
    red_color = [1 0 0];


    %% Data Plotting
    
    % ######################## Positions ########################
    % ++++++ Agent 3D Position ++++++
    % -- Prepare the data --
    % Destinations
    positions.x = positions.x;
    positions.y = positions.y;
    positions.z = positions.z;
    positions.dest_x = destinations(1,:)';
    positions.dest_y = zeros(1,n_agents);
    positions.dest_z = destinations(end,:)';
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ",num2str(agent))];
    end
    % Plot
    plot3D(positions, colors, legends, "Agent 3D Trajectories", pdfFileName, epsFilePrefix, showLegends, showFigures, true, saveEPS);

    % ++++++ Agents 2D trajectory ++++++
    for agent = 1:n_agents
        % Positions
        data.x = [positions.x(:,agent) linspace(destinations(1,agent),destinations(1,agent),n_steps)'];
        data.y = [positions.y(:,agent) linspace(0,0,n_steps)'];
        data.z = [positions.z(:,agent) linspace(destinations(end,agent),destinations(end,agent),n_steps)'];
        % Plot Colors
        plotColors = [colors(agent,:);red_color];
        % Labels
        labels = ["Time (s)"; "Position"];
        % Legends
        legends = [strcat("Agent ",num2str(agent)); strcat("Agent ",num2str(agent), " Destination")];
        % Plot Name
        plotName = strcat("Agent ", num2str(agent), " 2D Trajectories");
        % Plot
        plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
    end

    pose_Error_x_all = zeros(n_steps, n_agents);
    pose_Error_y_all = zeros(n_steps, n_agents);
    pose_Error_z_all = zeros(n_steps, n_agents);

    % ++++++ Separated Position Errors Unity ++++++
    for agent = 1:n_agents
        % Positions
        data.x = linspace(destinations(1,agent),destinations(1,agent),n_steps)' - positions.x(:,agent);
        data.y = 0 - positions.y(:,agent);
        data.z = linspace(destinations(end,agent),destinations(end,agent),n_steps)' - positions.z(:,agent);
        
        pose_Error_x_all(:,agent) = data.x;
        pose_Error_y_all(:,agent) = data.y;
        pose_Error_z_all(:,agent) = data.z;

        % Plot Colors
        plotColors = [colors(agent,:)];
        % Labels
        labels = ["Time (s)"; "Position Error"];
        % Legends
        legends = [strcat("Agent ",num2str(agent))];
        % Plot Name
        plotName = strcat("Agent ", num2str(agent), " Unity Position Error");
        % Plot
        plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
    end

    % ++++++ Joint Position Errors Unity ++++++
    % Positions
    data.x = pose_Error_x_all;
    data.y = pose_Error_y_all;
    data.z = pose_Error_z_all;
    % Plot Colors
    plotColors = colors;
    % Labels
    labels = ["Time (s)"; "Position Error"];
    % Legends
    legends = [];
    for agent= 1:n_agents
        legends = [legends; strcat("Agent ",num2str(agent))];
    end
    % Plot
    plot2D(data, times, plotColors, legends, labels, "Unity Position Errors", pdfFileName, epsFilePrefix, showLegends, showFigures, false, saveEPS);




    % ######################## Velocity ########################
    % ++++++ Agents Velocity ++++++
    for agent = 1:n_agents
        % Velocities
        data.x = vels.x(:,agent);
        data.y = vels.y(:,agent);
        data.z = vels.z(:,agent);
        % Plot Colors
        plotColors = [colors(agent,:)];
        % Labels
        labels = ["Time (s)"; "Velocity"];
        % Legends
        legends = [strcat("Agent ",num2str(agent))];
        % Plot Name
        plotName = strcat("Agent ", num2str(agent), " Velocity");
        % Plot
        plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
    end

    % ++++++ Joint Velocities Unity ++++++
    % Plot Colors
    plotColors = colors;
    % Labels
    labels = ["Time (s)"; "Velocities"];
    % Legends
    legends = [];
    for agent= 1:n_agents
        legends = [legends; strcat("Agent ",num2str(agent))];
    end
    % Plot
    plot2D(vels, times, plotColors, legends, labels, "Unity Velocities", pdfFileName, epsFilePrefix, showLegends, showFigures, false, saveEPS);

    
    % ######################## Acceleration ########################
    % ++++++ Accelerations Unity ++++++
    for agent = 1:n_agents
        % Accelerations
        data.x = accels.x(:,agent);
        data.y = accels.y(:,agent);
        data.z = accels.z(:,agent);
        % Plot Colors
        plotColors = [colors(agent,:)];
        % Labels
        labels = ["Time (s)"; "Acceleration"];
        % Legends
        legends = [strcat("Agent ",num2str(agent))];
        % Plot Name
        plotName = strcat("Agent ", num2str(agent), " Acceleration");
        % Plot
        plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
    end

    % ++++++ Joint Accelerations Unity ++++++
    % Plot Colors
    plotColors = colors;
    % Labels
    labels = ["Time (s)"; "Acceleration"];
    % Legends
    legends = [];
    for agent= 1:n_agents
        legends = [legends; strcat("Agent ",num2str(agent))];
    end
    % Plot
    plot2D(accels, times, plotColors, legends, labels, "Unity Accelerations", pdfFileName, epsFilePrefix, showLegends, showFigures, false, saveEPS);


    % ######################## Deformation Error (Gamma) ########################
    % ++++++ Unity Deformation Error ++++++
    % Plot Colors
    plotColors = [colors(1,:)];
    % Labels
    labels = ["Time (s)"; "Deformation Error (gamma)"];
    % Legends
    legends = ["gamma"];
    % Plot Name
    plotName = strcat("Deformation Error");
    % Plot
    plot2D(gamma_err, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS, true);
    
 
    % ######################## Position Error (eg) ########################
    % ++++++ Unity Position Error ++++++
    % Plot Colors
    plotColors = [colors(2,:)];
    % Labels
    labels = ["Time (s)"; "Position Error (eg)"];
    % Legends
    legends = ["eg"];
    % Plot Name
    plotName = strcat("Position Error");
    % Plot
    plot2D(eg_err, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS, true);
    
    % ######################## Scale Error (es) ########################
    % ++++++ Unity Scale Error ++++++
    % Plot Colors
    plotColors = [colors(3,:)];
    % Labels
    labels = ["Time (s)"; "Scale Error (es)"];
    % Legends
    legends = ["es"];
    % Plot Name
    plotName = strcat("Scale Error");
    % Plot
    plot2D(es_err, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS, true);
    
    % ######################## Rotation Error (eth) ########################
    % ++++++ Unity Scale Error ++++++
    % Plot Colors
    plotColors = [colors(4,:)];
    % Labels
    labels = ["Time (s)"; "Rotation Error (eth)"];
    % Legends
    legends = ["eth"];
    % Plot Name
    plotName = strcat("Rotation Error");
    % Plot
    plot2D(eth_err, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS, true);
  

    % ######################## Control Params ########################
    % ++++++ Deformation Control Value (U_D) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_D"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Deformation Control Value";
    % Plot
    plot2D(UD, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);

    % ++++++ Correction Term Deformation Control Value (U_G) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_G"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Correction Term Deformation Control Value";
    % Plot
    plot2D(UG, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Internal Deformation Control Value (U_H) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_G"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Internal Deformation Control Value";
    % Plot
    plot2D(UH, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  
    % ++++++ Shape Deformation Control Value (U_gamma) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_gamma"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Shape Deformation Control Value";
    % Plot
    plot2D(Ugamma, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false);
  

    % ++++++ Scale Control Value (U_s) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_s"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Scale Control Value";
    % Plot
    plot2D(Us, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  
    % ++++++ Position Control Value (U_g) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_g"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Position Control Value";
    % Plot
    plot2D(Ug, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Rotation Control Value (U_th) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_th"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Rotation Control Value";
    % Plot
    plot2D(Uth, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  
    % ++++++ Global Control Value (U_f) ++++++
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_f"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Global Control Value";
    % Plot
    plot2D(Uf, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  
    %% End
    disp(['Plots saved in A4 PDF: ' pdfFileName]);
