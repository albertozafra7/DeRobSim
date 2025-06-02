function [stresses_GL, eps_voigt, Ce] = computeStressesByDeformationGradientv2(...
    MeshTetrahedrons, VertexInitPoses, VertexCurrentPoses, Ce)

    N_tet = size(MeshTetrahedrons, 1);
    indices = MeshTetrahedrons; % N_tet x4
    % 
    % % Extract reference and current positions for all tetrahedrons
    % X0 = VertexInitPoses(indices(:,1), :); % N_tet x3
    % X1 = VertexInitPoses(indices(:,2), :);
    % X2 = VertexInitPoses(indices(:,3), :);
    % X3 = VertexInitPoses(indices(:,4), :);
    % 
    % x0 = VertexCurrentPoses(indices(:,1), :);
    % x1 = VertexCurrentPoses(indices(:,2), :);
    % x2 = VertexCurrentPoses(indices(:,3), :);
    % x3 = VertexCurrentPoses(indices(:,4), :);
    % 
    % % Compute Dm and Ds as 3x3xN_tet arrays
    % dX1 = X1 - X0; dX2 = X2 - X0; dX3 = X3 - X0;
    % dx1 = x1 - x0; dx2 = x2 - x0; dx3 = x3 - x0;
    % 
    % Dm = cat(2, reshape(dX1',3,1,[]), reshape(dX2',3,1,[]), reshape(dX3',3,1,[]));
    % Ds = cat(2, reshape(dx1',3,1,[]), reshape(dx2',3,1,[]), reshape(dx3',3,1,[]));
    % 
    % % Compute determinant of each Dm
    % detDm = Dm(1,1,:).*(Dm(2,2,:).*Dm(3,3,:) - Dm(2,3,:).*Dm(3,2,:)) ...
    %        - Dm(1,2,:).*(Dm(2,1,:).*Dm(3,3,:) - Dm(2,3,:).*Dm(3,1,:)) ...
    %        + Dm(1,3,:).*(Dm(2,1,:).*Dm(3,2,:) - Dm(2,2,:).*Dm(3,1,:));
    % detDm = detDm(:);
    % volumes = abs(detDm)' / 6;
    % 
    % if any(abs(detDm) < 1e-12)
    %     error('Inverted or degenerate element(s)');
    % end
    % 
    % % Compute inverse of Dm for each page
    % detDm_reshaped = reshape(detDm, 1,1,[]);
    % invDm = zeros(3,3,N_tet, 'like', Dm);
    % invDm(1,1,:) = (Dm(2,2,:).*Dm(3,3,:) - Dm(2,3,:).*Dm(3,2,:)) ./ detDm_reshaped;
    % invDm(1,2,:) = -(Dm(1,2,:).*Dm(3,3,:) - Dm(1,3,:).*Dm(3,2,:)) ./ detDm_reshaped;
    % invDm(1,3,:) = (Dm(1,2,:).*Dm(2,3,:) - Dm(1,3,:).*Dm(2,2,:)) ./ detDm_reshaped;
    % invDm(2,1,:) = -(Dm(2,1,:).*Dm(3,3,:) - Dm(2,3,:).*Dm(3,1,:)) ./ detDm_reshaped;
    % invDm(2,2,:) = (Dm(1,1,:).*Dm(3,3,:) - Dm(1,3,:).*Dm(3,1,:)) ./ detDm_reshaped;
    % invDm(2,3,:) = -(Dm(1,1,:).*Dm(2,3,:) - Dm(1,3,:).*Dm(2,1,:)) ./ detDm_reshaped;
    % invDm(3,1,:) = (Dm(2,1,:).*Dm(3,2,:) - Dm(2,2,:).*Dm(3,1,:)) ./ detDm_reshaped;
    % invDm(3,2,:) = -(Dm(1,1,:).*Dm(3,2,:) - Dm(1,2,:).*Dm(3,1,:)) ./ detDm_reshaped;
    % invDm(3,3,:) = (Dm(1,1,:).*Dm(2,2,:) - Dm(1,2,:).*Dm(2,1,:)) ./ detDm_reshaped;

    [Dm,Ds] = computeDeformationMatrices(MeshTetrahedrons,VertexInitPoses,VertexCurrentPoses);
    invDm = pageinv(Dm);
    % % Compute deformation gradient F = Ds * invDm
    F = pagemtimes(Ds, invDm);

    % Compute Green-Lagrange strain tensor: 0.5*(F'F - I)
    F_transposed = pagetranspose(F);
    FTF = pagemtimes(F_transposed, F);
    I = repmat(eye(3), 1, 1, N_tet);
    E_tensor = 0.5 * (FTF - I);

    % Convert to Voigt notation [E11; E22; E33; 2*E12; 2*E23; 2*E13]
    E11 = squeeze(E_tensor(1,1,:))';
    E22 = squeeze(E_tensor(2,2,:))';
    E33 = squeeze(E_tensor(3,3,:))';
    E12 = squeeze(E_tensor(1,2,:))';
    E23 = squeeze(E_tensor(2,3,:))';
    E13 = squeeze(E_tensor(1,3,:))';
    eps_voigt = [E11; E22; E33; 2*E12; 2*E23; 2*E13]; % 6xN_tet

    % Compute stresses
    stresses_GL = Ce * eps_voigt; % 6xN_tet
end


function [Dm, Ds] = computeDeformationMatrices(MeshTetrahedrons, VertexInitPoses, VertexCurrentPoses)
    % Number of tetrahedrons
    N_tet = size(MeshTetrahedrons, 1);
    
    % Total number of vertices
    N_vertices = size(VertexInitPoses, 1);
    
    % Initialize the selection matrix S
    % Each tetrahedron contributes 3 rows (for 3 edge vectors)
    % Each row has non-zero entries corresponding to the vertices involved in the edge
    rows = 3 * N_tet;
    cols = N_vertices;
    S = sparse(rows, cols);
    
    for i = 1:N_tet
        idx = MeshTetrahedrons(i, :);
        row_base = 3 * (i - 1);
        
        % Edge vector 1: X1 - X0
        S(row_base + 1, idx(2)) = 1;
        S(row_base + 1, idx(1)) = -1;
        
        % Edge vector 2: X2 - X0
        S(row_base + 2, idx(3)) = 1;
        S(row_base + 2, idx(1)) = -1;
        
        % Edge vector 3: X3 - X0
        S(row_base + 3, idx(4)) = 1;
        S(row_base + 3, idx(1)) = -1;
    end
    
    % Compute Dm and Ds
    Dm = S * VertexInitPoses;
    Ds = S * VertexCurrentPoses;
    
    % Reshape Dm and Ds into 3x3xN_tet arrays
    Dm = reshape(Dm', 3, 3, N_tet);
    Ds = reshape(Ds', 3, 3, N_tet);
end
