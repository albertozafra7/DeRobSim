% Computation of the tetrahedral meshNodes' new positions based on the displacements of
% the triangular mesh

% This function is just a preprocessing step of the internal forces computation
function [interp_meshNodes] = InterpolateMeshNodes(vertex_Initpose, vertex_Finalpose, meshNodes)

    % We compute the triangular mesh displacement
    vertex_Displacements = vertex_Finalpose - vertex_Initpose;

    % Number of nodes in the tetrahedral mesh
    n_meshNodes = size(meshNodes, 1);
    
    % Create the interpolation function based on the triangular mesh
    % displacements
    interpX = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,1), 'linear', 'nearest');
    interpY = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,2), 'linear', 'nearest');
    interpZ = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,3), 'linear', 'nearest');
    
    % Compute the tetrahedral mesh displacements
    meshDisplacements = zeros(n_meshNodes, 3);
    meshDisplacements(:,1) = interpX(meshNodes(:,1), meshNodes(:,2), meshNodes(:,3));
    meshDisplacements(:,2) = interpY(meshNodes(:,1), meshNodes(:,2), meshNodes(:,3));
    meshDisplacements(:,3) = interpZ(meshNodes(:,1), meshNodes(:,2), meshNodes(:,3));

    % Generate the new tetrahedral mesh nodes' positions
    interp_meshNodes = meshNodes + meshDisplacements;

end