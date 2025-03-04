function plot3D(data, colors, legends, name, pdfFileName, epsFilePrefix, showLegends, showFigures, firstFig, saveEPS)
    
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

    fig3D = figure;
    hold on;

    data_size = size(data.x);
    for i = 1:data_size(2)
        plot3(data.x(:, i), data.z(:, i), data.y(:, i), '-', 'Color', colors(i, :), 'DisplayName', legends(i));
        plot3(data.x(1, i), data.z(1, i), data.y(1, i), 'o', 'MarkerSize', 8, 'Color', colors(i, :), 'DisplayName', strcat(legends(i),' start'));  % Start marker
        plot3(data.x(end, i), data.z(end, i), data.y(end, i), '*', 'MarkerSize', 10, 'Color', colors(i, :), 'DisplayName', strcat(legends(i),' end'));  % End marker
        if(isfield(data,'dest_x') && isfield(data,'dest_z') && isfield(data,'dest_y'))
            n_dests = size(data.dest_x);
            if(i <= n_dests(1))
                plot3(data.dest_x(i), data.dest_z(i), data.dest_y(i), '+', 'MarkerSize', 12, 'Color', colors(i, :), 'LineWidth', 2, 'DisplayName', strcat(legends(i),' dest'));  % Destination marker
            end
        end
    end

    xlabel('X Axis');
    ylabel('Z Axis');
    zlabel('Y Axis');
    view(45, 50);
    grid on;
    axis padded;
    if showLegends
        legend('show');
    end
    hold off;

    % Store the 3D plot in the PDF
    if ~showFigures
        set(fig3D, 'Visible', 'off');
    end


    % Add a title annotation (to act as a section heading)
    annotation('textbox', [0.5 1 0 0], 'String', name, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14, 'FitBoxToText', 'on', 'EdgeColor', 'none');

    pdfFileName = strcat(pdfFileName, '.pdf');
    exportgraphics(fig3D, pdfFileName, 'Append', ~firstFig, 'ContentType', 'vector');
    if saveEPS
        name = regexprep(name, ' ', '_');
        epsFileName = strcat(epsFilePrefix, '_', name, '.eps');
        print(fig3D, '-depsc', epsFileName);
        disp(['EPS file saved as: ' epsFileName]);
    end
    close(fig3D);
    
end