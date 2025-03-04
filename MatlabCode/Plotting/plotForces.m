function [] = plotForces(VertexPose,MeshTetrahedrons,fe,fext,forcesSelect)

    if ~exist('forcesSelect','var')
        forcesSelect = "all";
    end

    % Plot Tetrahedral Mesh
    figure;
    tetramesh(MeshTetrahedrons, VertexPose, 'FaceAlpha', 0.3);
    hold on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Tetrahedral Mesh with Forces');
    
    if(contains(forcesSelect,"all",'IgnoreCase',true) || contains(forcesSelect,"int",'IgnoreCase',true))
        % Plot Internal Forces (e.g., blue arrows)
        internal = quiver3(VertexPose(:,1), VertexPose(:,2), VertexPose(:,3), ...
                fe(:,1), fe(:,2), fe(:,3), ...
                0, 'b', 'LineWidth', 1.5, 'DisplayName', 'Internal Forces');
    end

    if(contains(forcesSelect,"all",'IgnoreCase',true) || contains(forcesSelect,"ext",'IgnoreCase',true))
        % Plot External Forces (e.g., red arrows)
        external = quiver3(VertexPose(:,1), VertexPose(:,2), VertexPose(:,3), ...
                fext(:,1), fext(:,2), fext(:,3), ...
                0, 'r', 'LineWidth', 1.5, 'DisplayName', 'External Forces');
    
    end

    % Add Legend
    if(contains(forcesSelect,"all",'IgnoreCase',true))
        legend([internal,external]);
    elseif(contains(forcesSelect,"ext",'IgnoreCase',true))
        legend(external);
    else
        legend(internal);
    end
    grid on;
end