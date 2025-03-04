% Control of a team of mobile robots in a 3D working area with single-integrator dynamics.

% The goal of the controller is to keep, to the extent possible, the shape
% of the robots formation on its way from an initial to a desired configuration
% with different shape, size, centroid and rotation; while they suffer the least internal
% stress possible, avoiding to reach plastic deformations.

% The controller itself calculates the velocity for the robots to reach the
% desired position in each iteration.

% (Required) Inputs:
% vert_origins - Initial positions of the object's tetrahedral mesh vertices (N_vertsx3 matrix).
% vert_positions - Current positions of the object's tetrahedral mesh vertices (N_vertsx3 matrix).
% vert_prevpositions - Previous positions of the object's tetrahedral mesh vertices (N_vertsx3 matrix).
% agent_positions - Current positions of the agents (N_agentsx3 matrix).
% agent_destinations - Desired positions of the agents (N_agentsx3 matrix).
% meshTetrahedrons - Internal tetrahedral mesh relationship of the object's vertices (N_tetsx4 matrix).
% prev_stressTensor - Previous internal stress tensor (N_vertsx6 matrix).
% E - Elastic modulus (Young's modulus) of the material (scalar).
% nu - Poisson's coefficient of the material (scalar) [-1, +0.5].

% The code is based on the Final Master Thesis "Multi-robot control for
% transport of deformable objects," by R. Marcos-Saavedra 2024

% Author: Alberto Zafra Navarro, January 2025

% function [v, U_f, U_H, u_G, U_s, u_c, U_Hd, u_cbf, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual] = ForceControl3D_Debug(vert_origins, vert_positions, vert_prevpositions, agent_positions, agent_destinations, meshTetrahedrons, prev_stressTensor, E, nu, yield_stress, SF, kCBF, katt, krep, cbf_alpha, kH, kG, ks, kg, kth, sd, thd, vsat)
function [v, U_f, U_H, u_G, U_s, u_c, U_Hd, u_cbf, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual] = ForceControl3D_Debug(vert_origins, vert_positions, agent_positions, agent_destinations, meshTetrahedrons, J, E, nu, yield_stress, SF, kCBF, cbf_alpha, kH, kG, ks, kg, kth, sd, thd, vsat)

    % ++++++++++ Optional parameters evaluation ++++++++++

    % Previous stress tensor
    % if ~exist('prev_stressTensor', 'var')
    %     prev_stressTensor = zeros(size(vert_origins,2), 6);
    % end

    % Young Modulus
    if ~exist('E', 'var')
        E = 210e3;     % for computing the von Mises stress
    end

    % Poisson ratio
    if ~exist('nu', 'var')
        nu = 0.3;      % for computing the von Mises stress
    end

    % Yield Stress
    if ~exist('yield_stress', 'var')
        yield_stress = 200e3; % maximum stress where the object starts to being plastically deformed
    end

    % Safety factor
    if ~exist('SF', 'var')
        SF = 1.5; % safety factor usually used in structural mechanics [1.5 to 3]
    end

    % Stress-Minimization control gain
    if ~exist('kCBF', 'var')
        kCBF = 1;
    end

    % Goal Agent attraction control gain
    % if ~exist('katt', 'var')
    %     katt = 1;
    % end

    % Stress repulsive control gain
    % if ~exist('krep', 'var')
    %     krep = 1;      % for moving the mesh vertices away from the yield stress positions
    % end

    % Control Barrier function alpha
    if ~exist('cbf_alpha', 'var')
        cbf_alpha = 1; % for controling the convergence and the "safety" of the control barrier function
    end

    % Shape-Preserving control gain
    if ~exist('kH', 'var')
        kH = 1;        % for moving toward shape-preserving transformation
    end

    % Consistent configuration control gain
    if ~exist('kG', 'var')
        kG = 2;        % for moving toward a configuration consistent with our deformation modes
    end

    % Position control gain
    if ~exist('kg', 'var')
        kg = 0.5;      % for centroid translation
    end

    % Scale control gain
    if ~exist('ks', 'var')
        ks = 0.5;      % for scaling
    end

    % Orientation & scaling control gain
    if ~exist('kth', 'var')
        kth = 0.8;     % for scaling and orientation
    end

    % Target formation scale
    if ~exist('sd','var')
        sd = 1;
    end

    % Target formation rotation
    if ~exist('thd','var')
        thd = 0;
    end

    % Max robot velocity (in norm) allowed
    if ~exist('vsat', 'var')
        vsat = 1.5;    % m/s
    end
    


    % ++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    % ++++++++++ Properties Configuration ++++++++++

    % Number of positions and destinations
    N_pos = size(agent_positions, 2);
    N_des = size(agent_destinations, 2);
    assert(N_pos == N_des, 'Column size of current position (%i) and final position (%i) must be equal', N_pos, N_des);
    N = N_pos;

    % Number of tetrahedrons and nodes
    N_tet = size(meshTetrahedrons,1);
    N_verts = size(vert_origins,1);

    % We convert the position matrices into column arrays for a better computation
    c = reshape(agent_destinations,N*3,1); % (3*Na x 1)
    p = reshape(agent_positions,N*3,1); % (3*Na x 1)

    % We change the shape of the vertex positions into columns for
    % computing the stresses
    % ** Current Poses **
    currTetPoses = vert_positions(meshTetrahedrons',:);  % We get the poses of the vertices of each tetrahedron as (4*N_tet x 3)
    currTetPoses = permute(reshape(currTetPoses, 4, N_tet, 3), [1, 3, 2]); % We reshape the matrix to (4x3xN_tet) for a better understanding
    column_tet_positions = reshape(permute(currTetPoses, [2, 1, 3]), 12, 1, N_tet);  % We reshape to a column vector of (12x1xN_tet) in order to multiply it by the Rotation and stiffness matrices
    % ** Initial Poses **
    InitTetPoses = vert_origins(meshTetrahedrons',:); % (4*N_tet x 3)
    InitTetPoses = permute(reshape(InitTetPoses, 4, N_tet, 3), [1, 3, 2]); % (4x3xN_tet)
    column_tet_origins = reshape(permute(InitTetPoses, [2, 1, 3]), 12, 1, N_tet); % (12x1xN_tet)
        

    % ---------- Kabsch Algorithm ----------
    % We define the Kabsch Algorithm Matrices that we will use in our formulation
    K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix
    S = [0 -1; 1 0]; % 90 Deg rotation matrix
    % T = kron(eye(N),S);

    % We compute the position matrices using Kabsch Algorithm

    % -- Desired --
    Pdb = reshape(K * c, [3,N]);
    gd = compute_centroid(agent_destinations);

    % -- Current --
    Pb = reshape(K * p, [3,N]);
    g = compute_centroid(agent_positions);
    
    % We compute the deformation matrices
    % -- Matrix C_G (deformation modes -- linear, quadratic, mixed) --
    C_G = calculate_CG(N,K,c);        
     
    % -- Pseudoinverse ---
    C_Gp = pinv(C_G); % pseudoinverse
    % -- A_G???? --
    A_G = K - C_G * C_Gp; 

    % We compute the rotation matrices
    % -- Standard desired rotation matrix --
    Rd = eul2rotm([thd thd thd], 'XYZ');
    % Rd_T = kron(eye(N), Rd);
    % -- Scale and rotation matrix (for the control) --
    H_kd = sd * Rd; % we need this matrix to control the rotation

    % We compute the final rotation based on the desired positions

    W = eye(N); % as we considere w_i = 1, W is the identity matrix
    M_k = Pdb * W * Pb'; % covariant matrix
    % We apply the Singular Value Descomposition (SVD)
    [U_k, ~, V_k] = svd(M_k);
    
    % -- Destination Rotation matrix --
    I_k = eye(3);
    I_k(3,3) = det(V_k*U_k');
    R_k = V_k * I_k * U_k';
    
    % -- Destination Rotation angles --
    angles = rotm2eul(R_k, 'ZYX'); % in radians
    th_x = angles(3);
    th_y = angles(2);
    th_z = angles(1);
        
    % Other important parameters
    c_s = norm(Pdb, "fro")^2; % it can be compute as c_s = trace(C_b * C_b')
    % -- Current Scale factor --
    s_k = trace(Pb' * R_k * Pdb) / c_s; % current scale factor
    % -- Optimal Alignment Matrix --
    H_k = s_k * R_k;  % optimal alignment matrix

    % ------- 3D Internal Stress -------
    % Computation of the Elasticity Matrix (Material Stifness Matrix)
    Ce = CeMatrixComputation(E, nu); % 6x6 symmetric matrix

    % Computation of the strain-displacement matrix
    Le = LeMatrixComputation(meshTetrahedrons, vert_origins); % 6x12 matrix

    % Computed based on eq. 7 (Petit et all. 2017)
    % sigma_e = Ce * Le * u_hat_e
    % Ce_Le = pagemtimes(Ce, Le); % This results on a 6x12xN_tet matrix
    u_hat_e = column_tet_positions - column_tet_origins; % We compute the displacement of the nodes and store it in a column vector (12x1xN_tet)
    strains = pagemtimes(Le, u_hat_e); % Strain Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal strains and the 3 last are the shear strains
    stresses = pagemtimes(Ce, strains); % Stress Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal stresses (sigma) and the 3 last are the shear stresses (tau)
    
    % ------- von Mises Stress -------
    % We compute the vertex/tetrahedron-wise's von Mises Stress
    [curr_vmSigma, curr_SigmaTensor] = VonMisesStressComp(stresses, meshTetrahedrons, N_verts);


    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Control terms 
    % -- shape-preserving control (UH) --
    U_H = kH * (H_k * Pdb - Pb); % shape-preserving control
    u_H = reshape(U_H, [3*N,1]);
    
    % -- Deformation Control (UG) --
    u_G = -kG * A_G * p; % deformation control
    
    % -- Scale Control (US) --
    U_s = ks * (sd - s_k) * (1/s_k) * H_k * Pdb; % scale control
    u_s = reshape(U_s, [3*N,1]);
    
    % -- Position Control (Uc) --
    u_c = kg * kron(ones(N,1), gd - g); % translation control
    
    % -- Scale and Orientation Control (UHd) --
    U_Hd = kth * (H_kd * Pdb - Pb); % integrated scale and orientation control
    u_Hd = reshape(U_Hd, [3*N,1]);

    
    %%%%%%%%%%%%%%%%%%%%%%% 
  
    % Control law
    u = u_H + u_G + u_c + u_s + u_Hd;

    %%%%%%%%%%%%%%%%%%%%%%% 

    % -- Barrier Functions (Ucbf) --
    % U_cbf = kCBF * stressControlWithBarrier(vert_positions, vert_prevpositions, ...
    %                         agent_positions, agent_destinations, u, curr_SigmaTensor, curr_vmSigma, ...
    %                         prev_stressTensor, yield_stress, SF, cbf_alpha, katt, krep);
    U_cbf = kCBF * stressControlWithBarrier(3, u, curr_SigmaTensor, curr_vmSigma, ...
                            meshTetrahedrons, yield_stress, SF, cbf_alpha, Ce, Le, J);


    u_cbf = reshape(U_cbf, [3*N,1]);

    u = u_cbf;

    %%%%%%%%%%%%%%%%%%%%%%% 

    % Velocity saturation
    for i = 1:N
        if norm(u(3*i-2:3*i,:)) > vsat
            u(3*i-2:3*i,:) = vsat * u(3*i-2:3*i,:) / norm(u(3*i-2:3*i,:));
        end
    end

    v = reshape(u,3,N);

    % Debug:
    U_f = reshape(u,3,N);

    %Store current control errors
    gamma_H = (1/2) * norm((Pb - H_k * Pdb), "fro")^2;
    gamma_G = (1/2) * p' * A_G * p;
    eg = norm(g-gd); % centroid error
    es = s_k - sd;   % scale error
    
    e_H = eye(3) - R_k\Rd;
    eth = norm(e_H, "fro"); % rotation error
    eth_x = abs(th_x - thd);
    eth_y = abs(th_y - thd);
    eth_z = abs(th_z - thd);

    eth_individual = [eth_x;eth_y;eth_z];

 

end

function [g] = compute_centroid(P)
    N = size(P, 2);
    g = (1/N) * P * ones(N,1);
end


function C_G = calculate_CG(N,K,c)
    
% Initialization of each parameter
    l1 = zeros(3*N,1); % l: linear
    l2 = zeros(3*N,1);
    l3 = zeros(3*N,1);
    l4 = zeros(3*N,1);
    l5 = zeros(3*N,1);
    l6 = zeros(3*N,1);
    l7 = zeros(3*N,1);
    l8 = zeros(3*N,1);
    l9 = zeros(3*N,1);
    q1 = zeros(3*N,1); % q: quadratic
    q2 = zeros(3*N,1);
    q3 = zeros(3*N,1);
    q4 = zeros(3*N,1);
    q5 = zeros(3*N,1);
    q6 = zeros(3*N,1);
    q7 = zeros(3*N,1);
    q8 = zeros(3*N,1);
    q9 = zeros(3*N,1);
    m1 = zeros(3*N,1); % m: mixed
    m2 = zeros(3*N,1);
    m3 = zeros(3*N,1);
    m4 = zeros(3*N,1);
    m5 = zeros(3*N,1);
    m6 = zeros(3*N,1);
    m7 = zeros(3*N,1);
    m8 = zeros(3*N,1);
    m9 = zeros(3*N,1);

    for i = 1:N

        l1(3*i-2) = c(3*i-2);
        l2(3*i-2) = c(3*i-1);
        l3(3*i-2) = c(3*i);
        l4(3*i-1) = c(3*i-2);
        l5(3*i-1) = c(3*i-1);
        l6(3*i-1) = c(3*i);
        l7(3*i) = c(3*i-2);
        l8(3*i) = c(3*i-1);
        l9(3*i) = c(3*i);

        q1(3*i-2) = c(3*i-2).^2;
        q2(3*i-2) = c(3*i-1).^2;
        q3(3*i-2) = c(3*i).^2;
        q4(3*i-1) = c(3*i-2).^2;
        q5(3*i-1) = c(3*i-1).^2;
        q6(3*i-1) = c(3*i).^2;
        q7(3*i) = c(3*i-2).^2;
        q8(3*i) = c(3*i-1).^2;
        q9(3*i) = c(3*i).^2;

        m1(3*i-2) = c(3*i-2) * c(3*i-1);
        m2(3*i-2) = c(3*i-1) * c(3*i);
        m3(3*i-2) = c(3*i) * c(3*i-2);
        m4(3*i-1) = c(3*i-2) * c(3*i-1);
        m5(3*i-1) = c(3*i-1) * c(3*i);
        m6(3*i-1) = c(3*i) * c(3*i-2);
        m7(3*i) = c(3*i-2) * c(3*i-1);
        m8(3*i) = c(3*i-1) * c(3*i);
        m9(3*i) = c(3*i) * c(3*i-2);

    end
    
    C_G = K*[l1, l2, l3, l4, l5, l6, l7, l8, l9, ...
             q1, q2, q3, q4, q5, q6, q7, q8, q9, ...
             m1, m2, m3, m4, m5, m6, m7, m8, m9];  

end

%%%%%%%%%%%%%%%% Stress Computation %%%%%%%%%%%%%%%%

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

% Matrix used to change from 3D matrix to a Block Diagonal Matrix to
% perform faster the matrix multiplications
function [BlckDiagMtrx] = From3DToBlockDiagonalMatrix(matrix3D, toSparse)

    Kcell = cellfun(@sparse,  num2cell(matrix3D,[1,2]), 'uni',0  );
    BlckDiagMtrx=blkdiag(Kcell{:});

    if ~toSparse
        BlckDiagMtrx = full(BlckDiagMtrx);
    end
    
end

% Computation of the von Mises stress 
% source: https://en.wikipedia.org/wiki/Von_Mises_yield_criterion
function [VMSigma_n, Sigma_n] = VonMisesStressComp(stresses, meshTetrahedrons, N_verts)
    % Initialize node-based stress tensor
    % stress_n_disp = zeros(6,1,N_verts);
    % node_contributions = zeros(N_verts, 1); % Track contributions per node
    
    % Flatten MeshTetrahedrons for vectorized operations
    elements = meshTetrahedrons';  % Transpose to 4xN_tet
    elements = elements(:);        % Flatten to (4*N_tet x 1)
    
    % Reshape stresses to (6 x N_tet) for vectorized access
    stress_elem = reshape(stresses, 6, []);  % 6xN_tet

    % ---- Von Mises For Each Element ----
    VMSigma_e = zeros(length(meshTetrahedrons),1);

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
    
    % Get the stress tensor of each node
    Sigma_n = squeeze(stress_n_disp); % We convert the node tensor to (6xN_verts)
    Sigma_n = Sigma_n'; % (N_vertsx6)

    % Compute von Mises stress at each node
    VMSigma_n = zeros(N_verts, 1);
    
    sigma_12 = stress_n_disp(1,:,:) - stress_n_disp(2,:,:); % sigma_x - sigma_y
    sigma_23 = stress_n_disp(2,:,:) - stress_n_disp(3,:,:); % sigma_y - sigma_z
    sigma_31 = stress_n_disp(3,:,:) - stress_n_disp(1,:,:); % sigma_z - sigma_x

    VMSigma_n(:) = sqrt(0.5 .* (sigma_12.^2 + sigma_23.^2 + sigma_31.^2) + ...
        3 .* (stress_n_disp(4,:,:).^2 + stress_n_disp(5,:,:).^2 + stress_n_disp(6,:,:).^2));
    
    % ------------------

end


%%%%%%%%%%%%%%%% Barrier Functions %%%%%%%%%%%%%%%%

% function [AgentsVels] = stressControlWithBarrier(curr_VertexPose, prev_VertexPose, ...
%                             AgentsPose, AgentGoals, AgentActions, curr_StressTensor, curr_vmStress, ...
%                             prev_StressTensor, YieldStress, SF, alpha, Katt, Krep)

function [AgentsVels] = stressControlWithBarrier(ndim, AgentActions, curr_StressTensor, curr_vmStress, ...
                            meshTetrahedrons, YieldStress, SF, alpha, Ce, Be, J)
% stressControlWithBarrier
% This function simulates a control system where agents move towards goals while avoiding high-stress regions.
%
% Inputs:
% 1. curr_VertexPose: Current positions of nodal points (n_vertices x ndim)
% 2. prev_VertexPose: Previous positions of nodal points (n_vertices x ndim)
% 3. AgentsPose: Current positions of agents (n_agents x ndim)
% 4. AgentGoals: Desired positions of agents (n_agents x ndim)
% 5. AgentActions: Desired velocity of agents (n_agents x ndim)
% 6. curr_StressTensor: Current stress values for each virtual point (n_vertices x 6)
% 7. curr_vmStress: Current von Mises stress values for each virtual point (n_vertices x 1)
% 8. prev_StressTensor: Previous stress values for each virtual point (n_vertices x 6)
% 9. Yield: Yield allowed stress threshold (scalar)
% 10. SF: Safety factor used to compute the Maximum allowed stress threshold (scalar)
% 11. alpha: Barrier function constraint parameter (scalar)
% 12. Katt: Attraction gain for agents towards goals (scalar)
% 13. Krep: Repulsion gain for virtual points away from stress (scalar)
%
% Outputs:
% 1. AgentVels: Computed velocity of the agents after applying the barrier functions (n_agents x ndim)

    % Parameters
    % n_vertices = size(curr_VertexPose, 1); % Number of virtual points
    % n_agents = size(AgentsPose, 1);        % Number of agents
    % ndim = size(AgentsPose, 2);           % Number of dimensions

    % Preallocate velocities
    % AgentsVels = zeros(n_agents,ndim);
    % v_virtual = zeros(n_vertices,ndim);

    % Set optimization options
    options = optimset('Display', 'off');

    % Compute the MaxStress
    MaxStress = YieldStress/SF;

    % Get time measurement
    tStart = tic; % Start time


    % Barrier function control for virtual points
    % for i = 1:n_vertices
    % Compute stress barrier function
    h = MaxStress - curr_vmStress;
    % grad_h = computeVonMisesDerivativeFw(curr_StressTensor, curr_vmStress, Ce, Be, J, AgentActions, meshTetrahedrons);
    grad_h = computeVonMisesDerivative(curr_StressTensor, curr_vmStress, Ce, Be, J, meshTetrahedrons);

    % Desired velocity for repulsion from stress
    % vrep = Krep * (h / MaxStress);

    % Define QP problem
    H = 2 * eye(length(AgentActions));  % Quadratic cost for velocity
    f = -2 * AgentActions;              % Linear term for desired velocity
    A = grad_h;                         % Gradient of barrier function
    b = alpha * h;                      % Barrier constraint

    % Solve QP for velocity
    AgentsVels = quadprog(H, f, A, b, [], [], [], [], [], options);

    % end

    % % Aggregate virtual point influence for agents
    % for j = 1:n_agents
    %     % Compute agent's attraction velocity towards goal
    %     v_att = -Katt * (AgentsPose(j, :) - AgentGoals(j, :));
    % 
    %     % Aggregate virtual point repulsions based on proximity
    %     v_rep = zeros(1, ndim);
    %     for i = 1:n_vertices
    %         dist = norm(curr_VertexPose(i, :) - AgentsPose(j, :));
    %         if dist > 0
    %             weight = 1 / (dist^2); % Weight inversely proportional to squared distance
    %             v_rep = v_rep + weight * v_virtual(i,:);
    %         end
    %     end
    % 
    %     % Combine attraction and repulsion
    %     AgentsVels(j,:) = v_att + v_rep;
    % end
    
    tEnd = toc(tStart);
    fprintf("Computation completed after %f seconds.\n", tEnd);
end


%% Inverse computation (NOT Including the control action)
function grad_h = computeVonMisesDerivative(curr_stress, VM_stress, Ce, Le, J, meshTetrahedrons)

    N_tet = size(meshTetrahedrons, 1); % Number of tetrahedrons
    N_verts = size(VM_stress, 1);      % Number of vertices

    % -----------------------------------------------------------
    % 1️⃣ Compute von Mises stress gradient delta
    % -----------------------------------------------------------

    % Extract stress components
    sigma_11 = curr_stress(:, 1);
    sigma_22 = curr_stress(:, 2);
    sigma_33 = curr_stress(:, 3);
    sigma_12 = curr_stress(:, 4);
    sigma_13 = curr_stress(:, 5);
    sigma_23 = curr_stress(:, 6);

    % Compute derivatives of von Mises stress w.r.t. stress tensor components
    d_sigma_11 = (2 * sigma_11 - sigma_22 - sigma_33);
    d_sigma_22 = (-sigma_11 + 2 * sigma_22 - sigma_33);
    d_sigma_33 = (-sigma_11 - sigma_22 + 2 * sigma_33);
    d_sigma_12 = (6 * sigma_12);
    d_sigma_13 = (6 * sigma_13);
    d_sigma_23 = (6 * sigma_23);

    % Gradient of von Mises stress w.r.t. stress tensor components
    delta = [d_sigma_11, d_sigma_22, d_sigma_33, d_sigma_12, d_sigma_13, d_sigma_23]; % (6 x N_verts)

    % Scale delta by 2 / VM_stress (element-wise division)
    scaled_delta = delta ./ (2 * VM_stress); % (6 x N_verts)

    % -----------------------------------------------------------
    % 2️⃣ Construct matrix G: maps nodal displacements to element displacements
    % -----------------------------------------------------------

    elements = meshTetrahedrons';  
    elements = elements(:); % Flatten to (4*N_tet x 1)

    % Repeat each node index 3 times (for x, y, z DOFs)
    nodes_rep = repelem(elements, 3);  

    % Create column indices in global displacement vector (u)
    components = repmat([1; 2; 3], 4 * N_tet, 1);  
    columns = 3 * (nodes_rep - 1) + components;  

    % Row indices in G (sequential for 12 DOFs per tetrahedron)
    rows = (1:12*N_tet)';  

    % Sparse transformation matrix G (maps nodal displacements to element displacements)
    G = sparse(rows, columns, 1, 12*N_tet, 3*N_verts);

    % Compute element displacements
    element_displacements = G * J;  % (12*N_tet x 3*N_a)

    % -----------------------------------------------------------
    % 3️⃣ Compute element strains and stresses
    % -----------------------------------------------------------

    % Compute the Block Diagonal Strain-Displacement Matrix (6*N_tet x 12*N_tet)
    Le_blck = From3DToBlockDiagonalMatrix(Le, true);

    % Compute element strains (6 * N_tet x 3*N_a*N_tet)
    pred_strains = Le_blck * element_displacements;

    % Compute the Block Diagonal Stiffness Matrix (6*N_tet x 6*N_tet)
    Ce_blck = kron(eye(N_tet),Ce);
    Ce_blck = sparse(Ce_blck);

    % Compute element stresses (6*N_tet x 3*N_a*N_tet)
    pred_stresses_element = Ce_blck * pred_strains;

    % -----------------------------------------------------------
    % 4️⃣ Construct matrix W: maps element stresses to nodal stresses
    % -----------------------------------------------------------

    % Construct sparse mapping matrix W (6*N_verts x 6*N_tet)
    % Transpose and linearize node indices for rows
    rows = meshTetrahedrons';  % Transpose to 4×N_tet
    rows = rows(:);            % Flatten to 4*N_tet × 1

    % Column indices: [1,1,1,1,2,2,2,2,...,N_tet,N_tet,N_tet,N_tet]
    cols = repelem(1:N_tet, 4)';  % Repeat each tetrahedron index 4 times
    
    % Basic sparse matrix for node-element mapping (N_verts x N_tet)
    W_basic = sparse(rows, cols, 1, N_verts, N_tet);
    
    % Expand W to handle 6 stress components per node
    W_blck = kron(W_basic, speye(6)); % (6*N_verts x 6*N_tet)
    
    % Compute nodal stresses
    pred_nodal_stresses = W_blck * pred_stresses_element; % (6*N_verts x 3*N_a)
    
    % Normalize by the number of contributions per node
    denom = sum(W_blck, 2); % (6*N_verts x 3*N_a)
    pred_nodal_stresses = pred_nodal_stresses ./ denom;

    % -----------------------------------------------------------
    % 5️⃣ Compute the constraint matrix A
    % -----------------------------------------------------------

   % Create a cell array where each cell is a 1x6 row from the partial
   % derivatives
    cells = mat2cell(scaled_delta, ones(1, N_verts), 6);
    
    % Create S as a block-diagonal matrix to perform the multiplication
    % with the predicted stress
    S = blkdiag(cells{:}); % (N_verts x 6*N_verts)
    
    % Compute the final gradient of von Mises
    grad_h = S * pred_nodal_stresses; % (N_verts x 6*Nverts) * (6*N_verts x 3*N_a) -> (N_verts x 3*N_a)

end






%% Fordward computation (Including the control action)
function grad_h = computeVonMisesDerivativeFw(curr_stress, VM_stress, Ce, Le, J, u, meshTetrahedrons)

    N_tet = size(meshTetrahedrons,1);
    N_verts = size(VM_stress,1);

    sigma_11 = curr_stress(:,1);
    sigma_22 = curr_stress(:,2);
    sigma_33 = curr_stress(:,3);
    sigma_12 = curr_stress(:,4);
    sigma_13 = curr_stress(:,5);
    sigma_23 = curr_stress(:,6);

    % We compute the partial derivatives of the stress
    d_sigma_11 = (2*sigma_11 - sigma_22 - sigma_33);
    d_sigma_22 = (- sigma_11 + 2*sigma_22 - sigma_33);
    d_sigma_33 = (- sigma_11 - sigma_22 + 2*sigma_33);
    d_sigma_12 = (6*sigma_12);
    d_sigma_13 = (6*sigma_13);
    d_sigma_23 = (6*sigma_23);

    % delta = ds/dsigma (partial derivatives of the stress)
    delta = [d_sigma_11, d_sigma_22, d_sigma_33, d_sigma_12, d_sigma_13, d_sigma_23];

    % We compute the predicted stress tensor over the actions
    % Computed based on eq. 7 (Petit et all. 2017)
    % sigma_e = Ce * Le * u_hat_e
    % Ce_Le = pagemtimes(Ce, Le); % This results on a 6x12xN_tet matrix

    %%%% We generate a matrix G that would represent the transformation from nodal displacements to element displacements
    % Flatten tetrahedron node indices column-wise (matches MATLAB's linearization)
    elements = meshTetrahedrons'; % Transpose to 4×N_tet
    elements = elements(:);        % Flatten to column vector (4*N_tet × 1)
    
    % Repeat each node index 3 times (for x,y,z displacements)
    nodes_rep = repelem(elements, 3); % Now (12*N_tet × 1)
    
    % Create column indices in the global displacement vector (u)
    components = repmat([1; 2; 3], 4 * N_tet, 1); % [1,2,3,1,2,3,...] (12*N_tet × 1)
    columns = 3*(nodes_rep - 1) + components;      % Columns in u for x/y/z of each node
    
    % Row indices in G (sequential blocks of 12 per tetrahedron)
    rows = (1:12*N_tet)'; 
    
    % Sparse matrix construction
    G = sparse(rows, columns, 1, 12*N_tet, 3*N_verts);
    
    % Compute element displacements via matrix multiplication
    column_tet_disp = reshape(G * J * u, 12, 1, N_tet); % We reshape to a column vector of (12x1xN_tet) in order to multiply it by the Rotation and stiffness matrices (the 12 is 4 nodes x 3 dimensions)
    

    %%%% We compute the strains    
    pred_strains = pagemtimes(Le, column_tet_disp); % Strain Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal strains and the 3 last are the shear strains
    %%%% We compute the stresses
    pred_stresses_element = pagemtimes(Ce, pred_strains); % Stress Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal stresses (sigma) and the 3 last are the shear stresses (tau)

    %%%% We generate a matrix W that would represent the transformation between element stress and nodal stress
    % Transpose and linearize node indices for rows
    rows = meshTetrahedrons';  % Transpose to 4×N_tet
    rows = rows(:);            % Flatten to 4*N_tet × 1
    
    % Column indices: 1,1,1,1,2,2,2,2,...,N_tet,N_tet,N_tet,N_tet
    cols = repelem(1:N_tet, 4)';
    
    % Correct sparse matrix
    W = sparse(rows, cols, 1, N_verts, N_tet);

    % Get the stress tensor of each node
    pred_stresses = (squeeze(pred_stresses_element) * W') ./ (sum(W, 2)'); % We convert the node tensor to (6xN_verts)
    % pred_stresses = pred_stresses'; % (N_vertsx6)

    % We get the predicted von mises
    % pred_vmStress = delta * pred_stresses; % (N_verts x 1)
    pred_vmStress = sum(delta' .* pred_stresses, 1).'; % (N_verts x 1)
    grad_h = pred_vmStress ./ (2*VM_stress);
end

% function grad_h = computeVonMisesDerivativeFw(curr_stress, VM_stress, Ce, Le, J, u, meshTetrahedrons)
% 
%     N_tet = size(meshTetrahedrons,1);
%     N_verts = size(VM_stress,1);
% 
%     sigma_11 = curr_stress(:,1);
%     sigma_22 = curr_stress(:,2);
%     sigma_33 = curr_stress(:,3);
%     sigma_12 = curr_stress(:,4);
%     sigma_13 = curr_stress(:,5);
%     sigma_23 = curr_stress(:,6);
% 
%     % We compute the partial derivatives of the stress
%     d_sigma_11 = (2*sigma_11 - sigma_22 - sigma_33);
%     d_sigma_22 = (- sigma_11 + 2*sigma_22 - sigma_33);
%     d_sigma_33 = (- sigma_11 - sigma_22 + 2*sigma_33);
%     d_sigma_12 = (6*sigma_12);
%     d_sigma_13 = (6*sigma_13);
%     d_sigma_23 = (6*sigma_23);
% 
%     % delta = ds/dsigma (partial derivatives of the stress)
%     delta = [d_sigma_11, d_sigma_22, d_sigma_33, d_sigma_12, d_sigma_13, d_sigma_23];
% 
%     % We compute the predicted stress tensor over the actions
%     % Computed based on eq. 7 (Petit et all. 2017)
%     % sigma_e = Ce * Le * u_hat_e
%     % Ce_Le = pagemtimes(Ce, Le); % This results on a 6x12xN_tet matrix
%     nodal_displacements = J*u;
%     nodal_displacements = reshape(nodal_displacements, [3, N_verts])';
%     element_displacements  = nodal_displacements(meshTetrahedrons',:);
%     element_displacements = permute(reshape(element_displacements, 4, N_tet, 3), [1, 3, 2]); % We reshape the matrix to (4x3xN_tet) for a better understanding
%     column_tet_disp = reshape(permute(element_displacements, [2, 1, 3]), 12, 1, N_tet);  % We reshape to a column vector of (12x1xN_tet) in order to multiply it by the Rotation and stiffness matrices
%     pred_strains = pagemtimes(Le, column_tet_disp); % Strain Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal strains and the 3 last are the shear strains
%     pred_stresses_element = pagemtimes(Ce, pred_strains); % Stress Tensor of each tetrahedron (6x1xN_tet), where the 3 first components are the normal stresses (sigma) and the 3 last are the shear stresses (tau)
% 
%     % We compute the stress tensor per node
%     pred_nodal_stresses = zeros(6, 1, N_verts);    % Preallocate stress tensor per node
%     % Flatten MeshTetrahedrons for vectorized operations
%     elements = meshTetrahedrons';  % Transpose to 4xN_tet
%     elements = elements(:);        % Flatten to (4*N_tet x 1)
% 
%     % Accumulate stresses for all nodes (component-wise)
%     for k = 1:6
%         % Repeat stresses for all 4 nodes of each tetrahedron
%         stress_k_repeated = repelem(pred_stresses_element(k, :), 4)'; % Repeat component k for 4 nodes
%         % Accumulate the k-th component stresses
%         stress_accum_k = accumarray(elements, stress_k_repeated, [N_verts, 1]);
%         pred_nodal_stresses(k, 1, :) = reshape(stress_accum_k, 1, 1, []);
%     end
% 
%     % Count contributions per node
%     node_contributions = accumarray(elements, 1, [N_verts, 1]);
% 
%     % Average stress at each node
%     pred_nodal_stresses(:, :, node_contributions > 0) = pred_nodal_stresses(:, :, node_contributions > 0) ./ ...
%                                                   reshape(node_contributions(node_contributions > 0), 1, 1, []);
% 
%     % Get the stress tensor of each node
%     pred_stresses = squeeze(pred_nodal_stresses); % We convert the node tensor to (6xN_verts)
%     % pred_stresses = pred_stresses'; % (N_vertsx6)
% 
%     % We get the predicted von mises
%     % pred_vmStress = delta * pred_stresses; % (N_verts x 1)
%     pred_vmStress = sum(delta' .* pred_stresses, 1).'; % (N_verts x 1)
%     grad_h = pred_vmStress ./ (2*VM_stress);
% end
