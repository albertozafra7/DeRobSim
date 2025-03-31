n_agents =  size(u_Hs,2);
colors = rand(n_agents,3); 
times = linspace(0,0.005*niters,niters);
pdfFileName = "cubeTest";
epsFilePrefix =  "cubeTest";
saveEPS = false;
showFigures = true;

% ######################## Control Params ########################
    % ++++++ Deformation Control Value (U_H) ++++++
    % Deformation Control Value
    data.x = squeeze(u_Hs(1,:,:))';
    data.y = squeeze(u_Hs(2,:,:))';
    data.z = squeeze(u_Hs(3,:,:))';
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_H"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Deformation Control Value";
    % Plot
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Correction Term Deformation Control Value (U_G) ++++++
    % Correction Term Deformation Control Value
    data.x = squeeze(u_Gs(1,:,:))';
    data.y = squeeze(u_Gs(2,:,:))';
    data.z = squeeze(u_Gs(3,:,:))';
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
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Position Control Value (U_c) ++++++
    % Position Control Value
    data.x = squeeze(u_cs(1,:,:))';
    data.y = squeeze(u_cs(2,:,:))';
    data.z = squeeze(u_cs(3,:,:))';
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_c"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Position Control Value";
    % Plot
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Scale Control Value (U_s) ++++++
    % Scale Control Value
    data.x = squeeze(U_ss(1,:,:))';
    data.y = squeeze(U_ss(2,:,:))';
    data.z = squeeze(U_ss(3,:,:))';
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
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  

    % ++++++ Rotation and Scale Control Value (U_Hd) ++++++
    % Rotation and Scale Control Value
    data.x = squeeze(U_Hds(1,:,:))';
    data.y = squeeze(U_Hds(2,:,:))';
    data.z = squeeze(U_Hds(3,:,:))';
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_Hd"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Rotation and Scale Control Value";
    % Plot
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
  
    % ++++++ Global Control Value (U_f) ++++++
    % Global Control Value
    data.x = squeeze(u_cbfs(1,:,:))';
    data.y = squeeze(u_cbfs(2,:,:))';
    data.z = squeeze(u_cbfs(3,:,:))';
    % Plot Colors
    plotColors = colors(1:n_agents,:);
    % Labels
    labels = ["Time (s)"; " - U_cbf"];
    % Legends
    legends = [];
    for agent = 1:n_agents
        legends = [legends; strcat("Agent ", num2str(agent))];
    end
    % Plot Name
    plotName = "Barrier Function";
    % Plot
    plot2D(data, times, plotColors, legends, labels, plotName, pdfFileName, epsFilePrefix, false, showFigures, false, saveEPS);
