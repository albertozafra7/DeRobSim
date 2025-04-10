function [Le] = LeMatrixComputation(MeshTetrahedrons, VertexPose)
    % For each tetrahedron we have to compute its shape functions based on
    % its coefficients according to section 3.1 (Nesme et all. 2005)
    % combined with the natural coordinates change of (H. Ceric 2005).

    % Number of tetrahedrons
    N_tet = size(MeshTetrahedrons, 1);
    % Extract vertex indices for all tetrahedrons
    indices = MeshTetrahedrons';

    % Extract vertex coordinates for all tetrahedrons
    P = VertexPose(indices(:), :);
    P = reshape(P, 4, N_tet, 3);
    P = permute(P, [1, 3, 2]);

    % Compute shape function gradients for all tetrahedrons
    grads = computeShapeFunctionGradients(P);

    % Initialize Le matrix
    Le = zeros(6, 12, N_tet);

    % Assign gradients to Le matrix
    Le(1, 1:3:10, :) = grads(1, :, :);
    Le(2, 2:3:11, :) = grads(2, :, :);
    Le(3, 3:3:12, :) = grads(3, :, :);
    Le(4, 1:3:10, :) = grads(2, :, :);
    Le(4, 2:3:11, :) = grads(1, :, :);
    Le(5, 2:3:11, :) = grads(3, :, :);
    Le(5, 3:3:12, :) = grads(2, :, :);
    Le(6, 1:3:10, :) = grads(3, :, :);
    Le(6, 3:3:12, :) = grads(1, :, :);
end