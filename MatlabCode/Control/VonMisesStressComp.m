% Computation of the von Mises stress 
% source: https://en.wikipedia.org/wiki/Von_Mises_yield_criterion
function [VMSigma_n, VMSigma_e] = VonMisesStressComp(stresses, MeshTetraedrons, N_verts)
    % Initialize node-based stress tensor
    % stress_n_disp = zeros(6,1,N_verts);
    % node_contributions = zeros(N_verts, 1); % Track contributions per node
    
    % Flatten MeshTetrahedrons for vectorized operations
    elements = MeshTetraedrons';  % Transpose to 4xN_tet
    elements = elements(:);        % Flatten to (4*N_tet x 1)
    
    % Reshape stresses to (6 x N_tet) for vectorized access
    stress_elem = reshape(stresses, 6, []);  % 6xN_tet

    % ---- Von Mises For Each Element ----
    VMSigma_e = zeros(length(MeshTetraedrons),1);

    sigma_12 = stresses(1,:,:) - stresses(2,:,:); % sigma_x - sigma_y
    sigma_23 = stresses(2,:,:) - stresses(3,:,:); % sigma_y - sigma_z
    sigma_31 = stresses(3,:,:) - stresses(1,:,:); % sigma_z - sigma_x

    VMSigma_e(:) = sqrt(0.5 .* (sigma_12.^2 + sigma_23.^2 + sigma_31.^2) + ...
        3 .* (stresses(4,:,:).^2 + stresses(5,:,:).^2 + stresses(6,:,:).^2));


    % ---- Von Mises For Each Vertex ----
    % Initialize stress accumulation array
    stress_n_disp = zeros(6, 1, N_verts);    % Preallocate stress tensor per node
    
    % Accumulate stresses for all nodes (component-wise)
    for k = 1:6
        % Repeat stresses for all 4 nodes of each tetrahedron
        stress_k_repeated = repelem(stress_elem(k, :), 4)'; % Repeat component k for 4 nodes
        % Accumulate the k-th component stresses
        stress_accum_k = accumarray(elements, stress_k_repeated, [N_verts, 1]);
        stress_n_disp(k, 1, :) = reshape(stress_accum_k, 1, 1, []);
    end
    
    % Count contributions per node
    node_contributions = accumarray(elements, 1, [N_verts, 1]);

    
    % Average stress at each node
    
    stress_n_disp(:, :, node_contributions > 0) = stress_n_disp(:, :, node_contributions > 0) ./ ...
                                                  reshape(node_contributions(node_contributions > 0), 1, 1, []);
    
    % Compute von Mises stress at each node
    VMSigma_n = zeros(N_verts, 1);
    
    sigma_12 = stress_n_disp(1,:,:) - stress_n_disp(2,:,:); % sigma_x - sigma_y
    sigma_23 = stress_n_disp(2,:,:) - stress_n_disp(3,:,:); % sigma_y - sigma_z
    sigma_31 = stress_n_disp(3,:,:) - stress_n_disp(1,:,:); % sigma_z - sigma_x

    VMSigma_n(:) = sqrt(0.5 .* (sigma_12.^2 + sigma_23.^2 + sigma_31.^2) + ...
        3 .* (stress_n_disp(4,:,:).^2 + stress_n_disp(5,:,:).^2 + stress_n_disp(6,:,:).^2));
    
    % ------------------

    

end