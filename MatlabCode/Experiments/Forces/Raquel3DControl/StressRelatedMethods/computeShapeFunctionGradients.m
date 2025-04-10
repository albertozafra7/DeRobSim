function grads = computeShapeFunctionGradients(P)
    % Computes the gradients of the shape functions for a tetrahedron
    % Inputs:
    %   P: 4xN_tetx3 array with vertex coordinates for each tetrahedron
    % Outputs:
    %   grads: 4xN_tetx3 array of shape function gradients

    % Compute vectors for the Jacobian matrix
    V1 = squeeze(P(2, :, :) - P(1, :, :)); % N_tetx3 (Vector P1 -> P2)
    V2 = squeeze(P(3, :, :) - P(1, :, :)); % N_tetx3 (Vector P1 -> P3)
    V3 = squeeze(P(4, :, :) - P(1, :, :)); % N_tetx3 (Vector P1 -> P4)

    % Construct the Jacobian matrix J as 3x3xN_tet
    J = permute(cat(3, V1, V2, V3), [1, 3, 2]); % 3x3xN_tet

    % Compute determinants to check for singularity
    detJ = J(1,1,:).*J(2,2,:).*J(3,3,:) + ...
           J(1,2,:).*J(2,3,:).*J(3,1,:) + ...
           J(1,3,:).*J(2,1,:).*J(3,2,:) - ...
           J(1,3,:).*J(2,2,:).*J(3,1,:) - ...
           J(1,2,:).*J(2,1,:).*J(3,3,:) - ...
           J(1,1,:).*J(2,3,:).*J(3,2,:);

    if any(abs(detJ) < 1e-12, 'all')
        error('Jacobian is singular or nearly singular for one or more tetrahedrons. Check input points.');
    end

    % Compute the inverse of the Jacobian matrices using pageinv
    invJ = pageinv(J); % Result: 3x3xN_tet

    % Reference shape function gradients in local coordinates (ξ, η, ζ)
    local_grads = [-1, -1, -1;
                    1,  0,  0;
                    0,  1,  0;
                    0,  0,  1]'; % 3x4

    % Transform local gradients to global coordinates
    grads = pagemtimes(invJ,"transpose", local_grads,"none"); % Result: 3x4xN_tet

end