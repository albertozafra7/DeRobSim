function [] = plotMeshComp(original_vertices,deformed_vertices, MeshTetrahedrons, titles)
    % Visualize the original and deformed cylinder
    figure;
    
    % Original cylinder
    subplot(1, 2, 1);
    tetramesh(MeshTetrahedrons, original_vertices, 'FaceAlpha', 0.3);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(titles(1));
    
    % Deformed cylinder
    subplot(1, 2, 2);
    tetramesh(MeshTetrahedrons, deformed_vertices, 'FaceAlpha', 0.3);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(titles(2));
end