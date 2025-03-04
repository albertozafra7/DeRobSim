function [] = plotVerticesComp(original_vertices,deformed_vertices, titles)
    % Visualize the original and deformed cylinder
    figure;
    
    % Original cylinder
    subplot(1, 2, 1);
    scatter3(original_vertices(:, 1), original_vertices(:, 2), original_vertices(:, 3));
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(titles(1));
    
    % Deformed cylinder
    subplot(1, 2, 2);
    scatter3(deformed_vertices(:, 1), deformed_vertices(:, 2), deformed_vertices(:, 3));
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(titles(2));
end