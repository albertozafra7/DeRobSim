% Computation of external and internal forces applied to a deformable object
% based on the displacement of its vertices, as described in the work of 
% Petit et al. (2017).

% Inputs:
% VertexPose - Current positions of the object's vertices (Nx3 matrix).
% VertexPrevPose - Previous positions of the object's vertices (Nx3 matrix).
% MeshVolume - Volume of the mesh (scalar). ??
% Kext - Stiffness coefficient for external forces (scalar).
% E - Elastic modulus (Young's modulus) of the material (scalar).
% nu - Poisson's coefficient of the material (scalar).

% Outputs:
% fe - Internal elastic forces acting on the vertices due to deformation (Nx3 matrix).
% fext - External forces applied to the mesh from point cloud data or other sources (Nx3 matrix).

% References:
% --- 1 ---
% Petit, A., Lippiello, V., Fontanelli, G.A. and Siciliano, B. (2017) 
% 'Tracking elastic deformable objects with an RGB-D sensor for a pizza chef robot', 
% Robotics and Autonomous Systems, 88, pp. 187–201. 
% doi: 10.1016j.robot.2016.08.023.

% --- 2 ---
% Nesme, M., Payan, Y. and Faure, F. (2005)
% 'Efficient, physically plausible finite elements',
% https://inria.hal.science/inria-00394480

% --- 3 ---
% Hajdin CERIC (2005)
% 'Numerical Techniques in Modern TCAD',
% Dissertation, Technische Universität Wien, 2005. 
% https://www.iue.tuwien.ac.at/phd/orio/node48.html
function [fe, fext, decoupled_fe, stresses] = ForcesComputation(VertexPose, VertexPrevPose, VertexInitPose, MeshTetraedrons, MeshVolumes, Kext, E, nu)

    % ++++++++++ Optional parameters evaluation ++++++++++


    % ++++++++++++++++++++++++++++++++++++++++++++++++++++

    % ++++++++++ Properties Configuration ++++++++++
    
    % Number of vertices
    N_verts = length(VertexPose);
    % Numbert of tetrahedrons
    N_tet = length(MeshTetraedrons);


    % ++++++++++++++++++++++++++++++++++++++++++++++++++++

    % ++++++++++ External Forces Computation ++++++++++
    
    % We compute the external pose based on eq. 10 (Petit et all. 2017) 
    fext = Kext * (VertexPose - VertexPrevPose);

    % We can add a weight based on the contour --> check eqs. 12 and 13 (Petit et all. 2017)
    % TODO

    % ++++++++++++++++++++++++++++++++++++++++++++++++++++

    % ++++++++++ Internal Forces Computation ++++++++++

    % Computation of the Elasticity Matrix (Material Stifness Matrix)
    Ce = CeMatrixComputation(E, nu); % 6x6 symmetric matrix

    % Computation of the strain-displacement matrix
    Le = LeMatrixComputation(MeshTetraedrons, VertexInitPose); % 6x12 matrix

    % --- Ke ---
    % Computation of the stiffness matrix based on eq. 8 (Petit et all. 2017)
    % 12x12 = scalar * 12x6 * 6x6 * 6x12
    %  Ke = MeshVolumes * Le' * Ce * Le;
    
    % Avg diff of 1e-16
    % for e=1:N_tet
    %   Ke(:,:,e) = MeshVolumes(e) * Le(:,:,e)' * Ce * Le(:,:,e);
    % end

    % Extract Le' (transpose of Le) across all tetrahedrons
    Le_transpose = permute(Le, [2, 1, 3]);  % This will make it 12x6xN_tet
    % Perform matrix multiplication for all tetrahedrons at once
    % First, multiply Le' * Ce
    Le_Ce = pagemtimes(Le_transpose, Ce);  % Performs matrix multiplication Le' * Ce for each tetrahedron
    % Then, multiply the result by Le for each tetrahedron
    Le_Ce_Le = pagemtimes(Le_Ce, Le);  % Performs matrix multiplication (Le' * Ce) * Le for each tetrahedron
    % Multiply the result by the Volume of the tetrahedron
    Ke = bsxfun(@times, Le_Ce_Le, reshape(MeshVolumes, 1, 1, []));  % This performs matrix multiplication on 3D matrices resulting in the stiffness matrix for all tetrahedrons (a 12x12xN_tet matrix)
    % -----------------
    

    % Computation of the deformation rotation matrix based on the QR
    % decomposition (Nesme et all. 2005)
    Re = RotationComputation(VertexPose, VertexInitPose, MeshTetraedrons);

    % ---- fe ----
    % Computation of the internal forces based on eq. 9 (Petit et all. 2017)

    % We create the resulting internal forces' matrix
    fe = zeros(N_verts,3);

    % We convert the vertex poses into a column vector for multiplying it
    % to the rotation and stiffness matrices
    
    % ** Current Poses **
    currTetPoses = VertexPose(MeshTetraedrons',:);  % We get the poses of the vertices of each tetrahedron as (4*N_tet x 3)
    currTetPoses = permute(reshape(currTetPoses, 4, N_tet, 3), [1, 3, 2]); % We reshape the matrix to (4x3xN_tet) for a better understanding
    columnCurrPoses = reshape(permute(currTetPoses, [2, 1, 3]), 12, 1, N_tet);  % We reshape to a column vector of (12x1xN_tet) in order to multiply it by the Rotation and stiffness matrices
    % ** Initial Poses **
    InitTetPoses = VertexInitPose(MeshTetraedrons',:); % (4*N_tet x 3)
    InitTetPoses = permute(reshape(InitTetPoses, 4, N_tet, 3), [1, 3, 2]); % (4x3xN_tet)
    columnInitPoses = reshape(permute(InitTetPoses, [2, 1, 3]), 12, 1, N_tet); % (12x1xN_tet)

    % ** Internal forces computation **
    poseIncr = (pagemtimes(pageinv(Re),columnCurrPoses) - columnInitPoses); % We compute the unrotated increment of the vertices' pose
    Ke_pose = pagemtimes(Ke,poseIncr); % We multiply by the stiffness matrix
    columnFe = pagemtimes(Re,Ke_pose); % We compute the tetrahedrals' internal forces (12x1xN_tet)

    currFe = permute(reshape(columnFe, 3, 4, N_tet), [2, 1, 3]); % We reshape the matrix to (4x3xN_tet) for a better understanding
    decoupled_fe = currFe;

    % We have to compute the global sum of the internal forces applied to
    % each vertex
    % Reshape the computed force to (N_tet*4)x3 (combine slices along the rows)
    currFe = reshape(permute(currFe, [1, 3, 2]), [], 3); % (N_tet*4)x3
    
    % Flatten the tetrahedron indices to 48x1 (repeat indices for accumulation)
    MeshTetraedrons_flat = MeshTetraedrons';
    MeshTetraedrons_flat = MeshTetraedrons_flat(:); % (N_tet*4)x1
    
    % Accumulate forces using accumarray
    fe(:,1) = accumarray(MeshTetraedrons_flat, currFe(:, 1), [size(fe, 1), 1]); % x-component
    fe(:,2) = accumarray(MeshTetraedrons_flat, currFe(:, 2), [size(fe, 1), 1]); % y-component
    fe(:,3) = accumarray(MeshTetraedrons_flat, currFe(:, 3), [size(fe, 1), 1]); % z-component
    % -----------------

    % ----- Stresses -----
    % Computed based on eq. 7 (Petit et all. 2017)
    % sigma_e = Ce * Le * u_hat_e
    % Ce_Le = pagemtimes(Ce, Le); % This results on a 6x12xN_tet matrix
    u_hat_e = columnCurrPoses - columnInitPoses; % We compute the displacement of the nodes and store it in a column vector (12x1xN_tet)
    strains = pagemtimes(Le, u_hat_e); % Strain Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal strains and the 3 last are the shear strains
    stresses = pagemtimes(Ce, strains); % Stress Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal stresses (sigma) and the 3 last are the shear stresses (tau)
    
end

% Computation of the Elasticity Matrix (Material Stifness Matrix) from
% Hooke's law for 3D isotropic materials
function [Ce] = CeMatrixComputation(E, nu)

    poissonMatrix = [1-nu nu nu 0 0 0;...
                      nu 1-nu nu 0 0 0;...
                      nu nu 1-nu 0 0 0;...
                      0 0 0 (1-2*nu)/2 0 0;...
                      0 0 0 0 (1-2*nu)/2 0;...
                      0 0 0 0 0 (1-2*nu)/2];

    Ce = E/((1+nu)*(1-2*nu)) * poissonMatrix;

end


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


% Computation of the rigid rotation based on QR decomposition (Nesme et all. 2005)
function [Re] = RotationComputation(VertexPose, VertexInitPose, MeshTetraedrons)

    % We intialize the returning matrix
    N_tet = length(MeshTetraedrons);
    Re = zeros(12,12,N_tet);

    % The vertices that compose the faces are the following
    faces = [1 2 3;...
             1 2 4;...
             1 3 4;...
             2 3 4];

    % Hence for each tetrahedron
    for e = 1:length(MeshTetraedrons)
        % We store the initial and current vertex poses
        currPoses = [VertexPose(MeshTetraedrons(e,1),:);VertexPose(MeshTetraedrons(e,2),:);VertexPose(MeshTetraedrons(e,3),:);VertexPose(MeshTetraedrons(e,4),:)];
        initPoses = [VertexInitPose(MeshTetraedrons(e,1),:);VertexInitPose(MeshTetraedrons(e,2),:);VertexInitPose(MeshTetraedrons(e,3),:);VertexInitPose(MeshTetraedrons(e,4),:)];
        
        % We compute its gradient
        delta_X = currPoses-initPoses;

        % We obtain the qr decomposition of each face
        for f = 1:length(faces)
            [Re(1+3*(f-1):3*f,1+3*(f-1):3*f,e),~] = qr(delta_X(faces(f,:),:));
        end

        
    end
   
end


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