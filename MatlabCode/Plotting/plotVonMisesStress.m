% Visualize the von Mises stress with a heat map
function [] = plotVonMisesStress(MeshTetrahedrons, VertexPose,VMSigma_n, plot_title)

    if ~exist("plot_title", 'var')
        plot_title = "";
    end
    
    % Extract surface triangles from the tetrahedral mesh
    faces = [
        MeshTetrahedrons(:, [1, 2, 3]);
        MeshTetrahedrons(:, [1, 2, 4]);
        MeshTetrahedrons(:, [1, 3, 4]);
        MeshTetrahedrons(:, [2, 3, 4])
    ];
    % faces = unique(sort(faces, 2), 'rows'); % Remove duplicate faces
    VMSigma_n_log = log10(VMSigma_n + 1e-6); % Offset to avoid log(0)

    % Plot the mesh with node-based von Mises stress values
    figure;
    trisurf(faces, VertexPose(:, 1), VertexPose(:, 2), VertexPose(:, 3), VMSigma_n, 'EdgeColor', 'none');
    % trisurf(faces, VertexPose(:, 1), VertexPose(:, 2), VertexPose(:, 3), VMSigma_n_log, 'EdgeColor', 'none');
        
    colormap('jet'); % Use jet colormap for stress values
    colorbar; % Display color scale
    plot_title = strcat("Heat Map of von Mises Stress (MPa) ", plot_title);
    title(plot_title);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    view(3); % Set 3D view


end