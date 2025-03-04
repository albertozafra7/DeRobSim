function plot2D(data, times, colors, legends, labels, name, pdfFileName, epsFilePrefix, showLegends, showFigures, firstFig, saveEPS, individual_plot)

    if ~exist("showLegends",'var')
        showLegends = true;
    end

    if ~exist("showFigures", 'var')
        showFigures = false;
    end

    if ~exist("firstFig", 'var')
        firstFig = false;
    end

    if ~exist("saveEPS", 'var')
        saveEPS = false;
    end

    if ~exist("individual_plot", 'var')
        individual_plot = false;
    end



    fig2D = figure;
    hold on;
    if individual_plot
        data_size = size(data);
        for i = 1:data_size(2)
            plot(times, data(:,i),'-', 'Color', colors(i,:), 'DisplayName', legends(i));
        end
        hold off;
        xlabel(labels(1));
        ylabel(labels(2));
        grid on;
        axis padded;
        if showLegends
            legend("show");
        end
    
    else
        data_size = size(data.x);
        subplot(3, 1, 1);
        hold on;
        for i = 1:data_size(2)
            plot(times, data.x(:, i), '-', 'Color', colors(i, :), 'DisplayName', legends(i));
        end
        hold off;
        xlabel(labels(1));
        ylabel(['X ' labels(2)]);
        grid on;
        axis padded;
        if showLegends
            legend("show");
        end

        subplot(3, 1, 2);
        hold on;
        for i = 1:data_size(2)
            plot(times, data.y(:, i), '-', 'Color', colors(i, :), 'DisplayName', legends(i));
        end
        hold off;
        xlabel(labels(1));
        ylabel(['Y ' labels(2)]);
        grid on;
        axis padded;
        if showLegends
            legend("show");
        end

        subplot(3, 1, 3);
        hold on;
        for i = 1:data_size(2)
            plot(times, data.z(:, i), '-', 'Color', colors(i, :), 'DisplayName', legends(i));
        end
        hold off;
        xlabel(labels(1));
        ylabel(['Z ' labels(2)]);
        grid on;
        axis padded;

        if showLegends
            legend("show");
        end
    end

    

    if ~showFigures
        set(fig2D, 'Visible', 'off');
    end

    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', name, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    % Store the 2D subplots in the PDF
    pdfFileName = strcat(pdfFileName, '.pdf');
    exportgraphics(fig2D, pdfFileName, 'Append', ~firstFig, 'ContentType', 'vector');
    if saveEPS
        name = regexprep(name, ' ', '_');
        epsFileName = strcat(epsFilePrefix, '_', name, '.eps');
        print(fig2D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);
    end

    close(fig2D);
end