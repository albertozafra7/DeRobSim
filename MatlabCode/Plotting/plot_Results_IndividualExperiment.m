function [] = plot_Results_IndividualExperiment(results,plot_CBF_VelCorr,plot_CBF_VelDiff, plot_VelComp,plot_VonMisesMax,plot_VMStressComp)
    % Seting up the stress colorbar
    % Define proportions for each color phase
    nColors = 256;
    g = round(0.10 * nColors);   % Green
    y = round(0.20 * nColors);   % Yellow
    o = round(0.30 * nColors);   % Orange
    r = nColors - g - y - o;     % Red
    
    % Define color transitions
    green   = [linspace(0,1,g)', ones(g,1), zeros(g,1)]; % dark → green
    yellow  = [ones(y,1), linspace(1,1,y)', zeros(y,1)]; % green → yellow
    orange  = [ones(o,1), linspace(1,0.5,o)', zeros(o,1)]; % yellow → orange
    red     = [ones(r,1), linspace(0.5,0,r)', zeros(r,1)]; % orange → red
    
    % Combine the segments
    customMap = [green; yellow; orange; red];

    plotaxfont='Helvetica';
    color_robots = ["#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7", "#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7"]; % 14 robots
    color_graphs = ["#0073bd", "#77a5c3", "#d9541a", "#edb120", "#7e2f8e"];

    %% Individual Velocity Correction plot
    if plot_CBF_VelCorr
        % LINEAR VELOCITY IN X
        % We create the figure and set the parameters
        figure
        % subplot(3,1,1);
        set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
            plot(results.prop_vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
            % plot(vs(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity in X (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity X Comp', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY IN Y
        % We create the figure and set the parameters
        % subplot(3,1,2);
        figure;
        set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
            plot(results.prop_vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
            % plot(vs(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity in Y (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Y Comp', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY IN Z
        % We create the figure and set the parameters
        % subplot(3,1,3);
        figure
        set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
            plot(results.prop_vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
            % plot(vs(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity in Z (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Z Comp', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY NORM
        % We create the figure and set the parameters
        figure
        set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        
        % Representation
        for i = 1:results.N
            plot(results.nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
            plot(results.prop_nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
            % plot(nvs(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity norm (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Norm Comp', 'FontSize', 14);
        hold off;
    
    end
    
    %% Velocity Differences
    
    if plot_CBF_VelDiff
        % LINEAR VELOCITY IN X
        % We create the figure and set the parameters
        figure
        subplot(3,1,1);
        % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.prop_vs_cbf(3*i-2,:) - results.vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity diff in X (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity X Difference', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY IN Y
        % We create the figure and set the parameters
        subplot(3,1,2);
        % figure;
        % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.prop_vs_cbf(3*i-1,:) - results.vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity diff in Y (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        % legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Y Difference', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY IN Z
        % We create the figure and set the parameters
        subplot(3,1,3);
        % figure
        % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        grid on
        
        % Representation
        for i = 1:results.N
            plot(results.prop_vs_cbf(3*i,:) - results.vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity diff in Z (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        % legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Z Difference', 'FontSize', 14);
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % LINEAR VELOCITY NORM
        % We create the figure and set the parameters
        figure
        set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
        box on
        hold on
        
        % Representation
        for i = 1:results.N
            plot(results.prop_nvs_cbf(i,:) - results.nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        end
        
        % Titles and dimensions
        set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
        xlabel('Time (s)', 'FontSize', 18)
        ylabel('Linear velocity norm (m/s)', 'FontSize', 18)
        
        xlim([0, results.niters]);
        
        % legend
        legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
        
        % Title
        title('Linear Velocity Norm Difference', 'FontSize', 14);
        hold off;
    
    end
    
    %% Velocity comparation
    
    if plot_VelComp
    
        figure
        plot(results.nvs_3D(1,:)');
        hold on;
        plot(results.nvs_cbf(1,:)');
        % if use_real_CBF
            plot(results.prop_nvs_cbf(1,:)');
        % end
        hold off;
        title("Velocity Norm Comparison");
        xlabel("Iters");
        ylabel("m/s");
        % legend(["Agend 1 - std", "Agend 2 - std", "Agend 3 - std", "Agend 4 - std", "Agend 1 - cbf", "Agend 2 - cbf", "Agend 3 - cbf", "Agend 4 - cbf"]);
        % if use_real_CBF
            legend(["Agend 1 - std", "Agend 1 - cbf", "Agent 1 - Proposed cbf"]);
        % else
            % legend(["Agend 1 - std", "Agend 1 - cbf"]);
        % end
    
    end
    
    %% Plot von Mises maximum ------> Curr VM
    if plot_VonMisesMax
        figure;
        hold on;
        for a = 1:results.N
            plot(max(results.vm_stresses_cbf),'Color','b');
        end
        hold off;
        title("Maximum Von Mises per iteration");
        xlabel("Iters");
        ylabel("von Mises Stress (PA)");
        grid on;
        legend("MaxVonMises");
    
    end
    
    
    %% Plot VM Stress Comparison
    if plot_VMStressComp
        % This plot the differences between the maximum value of the von Mises
        % stress during the simulation
        figure;
        plot(max(results.vm_stresses_cbf),'Color','b','DisplayName', "CBF - Max VMStress (σ_c_u_r_r)", 'linewidth', 2);
        hold on;
        plot(repelem(results.yield_stress/results.SF,size(results.vm_stresses_cbf,2)),'Color', 'r', 'DisplayName', "Yield Stress (σ_y_i_e_l_d)", 'linewidth', 2);
        plot(max(results.pred_vmStresses_cbf),'Color','g','DisplayName',"CBF - Max Pred VMStres (σ_p_r_e_d)", 'linewidth', 2);
        plot(max(results.vm_stresses_3D),'Color',[1 0.5 0],'DisplayName',"3D Control - Max VMStress (σ_3_D)", 'linewidth',2);
        hold off;
        xlabel('Iteration');
        ylabel('VM Stress');
        title('Von Mises Stress Comparison');
        legend show;
        grid on;
    end

end