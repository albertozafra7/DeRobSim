function [] = plotErrorResults(e_max, e_min, e_mean, e_std, plotPrefix)
    % Change underscore with space
    plotPrefix = strrep(plotPrefix, '_', ' ');
    n = length(e_max);
    % Plot Over Time
    figure;
    plot(1:n, e_max, 'r', ...
         1:n, e_min, 'b', ...
         1:n, e_mean, 'g', ...
         1:n, e_std, 'k');
    legend('Max', 'Min', 'Mean', 'Std');
    xlabel('Time step');
    ylabel(strcat(plotPrefix,' Error'));
    title(strcat(plotPrefix,' Error Metrics Over Time'));
    grid on;
    
    %Histogram of Each Metric 
    figure;
    subplot(2,2,1); histogram(e_max); title(strcat("Max ", plotPrefix," Error"));
    subplot(2,2,2); histogram(e_min); title(strcat("Min ", plotPrefix," Error"));
    subplot(2,2,3); histogram(e_mean); title(strcat("Mean ", plotPrefix," Error"));
    subplot(2,2,4); histogram(e_std); title(strcat("Std ", plotPrefix," Error"));
end