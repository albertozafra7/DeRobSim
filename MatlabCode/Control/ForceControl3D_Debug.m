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
function [v, U_f, u_3D, U_H, u_G, U_s, u_c, U_Hd, u_cbf, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual, curr_vmSigma, curr_SigmaTensor, stresses, strains, A, b, grad_h, scaled_delta, pred_nodal_stresses, pred_stresses_element, pred_strains, element_displacements] = ForceControl3D_Debug(vert_origins, vert_positions, agent_positions, agent_destinations, meshTetrahedrons, J, Ce, Le, max_stress, kCBF, cbf_alpha, kH, kG, ks, kg, kth, sd, thd, vsat)

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
    % N_tet = size(meshTetrahedrons,1);
    N_verts = size(vert_origins,1);

    % +++++++++++++++++++++++++++++++++++
    % ++++++++++ 3D Controller ++++++++++
    % +++++++++++++++++++++++++++++++++++
    [u_3D, ~, U_H, u_G, U_s, u_c, U_Hd, ~, ~, gamma_H, gamma_G, eg, es, eth, eth_individual] = TransportationControl3D_Debug(agent_positions, agent_destinations, kH, kG, ks, kg, kth, sd, thd, vsat);
    u_3D = u_3D(:);

    % +++++++++++++++++++++++++++++++++++
    % +++++++ 3D Internal Stress ++++++++
    % +++++++++++++++++++++++++++++++++++
    %%%%%%% NEW METHOD %%%%%%%
    % [stresses, strains, ~] = computeStressesByDeformationGradientv2(meshTetrahedrons, vert_origins, vert_positions, Ce);
    [stresses, ~, strains, ~,~] = ComputeStressByDisplacements(vert_origins - vert_positions,meshTetrahedrons, Ce, Le);


    % ------- von Mises Stress -------
    % We compute the vertex/tetrahedron-wise's von Mises Stress
    [curr_vmSigma, ~, curr_SigmaTensor, ~] = VonMisesStressComp(stresses, meshTetrahedrons, N_verts);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % -- Barrier Functions (Ucbf) --
    % U_cbf = kCBF * stressControlWithBarrier(u, curr_SigmaTensor, curr_vmSigma, ...
    %                         meshTetrahedrons, yield_stress, SF, cbf_alpha, Ce, Le, J);

    [U_cbf, A, b, grad_h, scaled_delta, pred_nodal_stresses, pred_stresses_element, pred_strains, element_displacements] = stressControlWithBarrier_Debug(u_3D, curr_SigmaTensor', curr_vmSigma, ...
                            meshTetrahedrons, max_stress, cbf_alpha, kCBF, Ce, Le, J);


    u_cbf = reshape(U_cbf, [3*N,1]);

    u = u_cbf;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    % Velocity saturation
    for i = 1:N
        if norm(u(3*i-2:3*i,:)) > vsat
            u(3*i-2:3*i,:) = vsat * u(3*i-2:3*i,:) / norm(u(3*i-2:3*i,:));
        end
    end

    v = reshape(u,3,N);

    % Debug:
    U_f = reshape(u,3,N);
 

end


%%%%%%%%%%%%%%%% Barrier Functions %%%%%%%%%%%%%%%%

function [AgentsVels, A, b, grad_h, scaled_delta, pred_nodal_stresses, ...
          pred_stresses_element, pred_strains, element_displacements] = ...
          stressControlWithBarrier_Debug(AgentActions, curr_StressTensor, curr_vmStress, ...
                                         meshTetrahedrons, MaxStress, alpha, Kcbf, Ce, Le, J)
    % stressControlWithBarrier_Debug
    %   Solves a QP of the form:
    %       minimize    (u - u_des)'*(u - u_des)   subject to   -grad_h * u <= alpha*h
    %
    %   Inputs:
    %     - AgentActions:   (3N_agents×1) “desired” velocity vector (u_des)
    %     - curr_StressTensor:  (N_verts×6) matrix of current stress components at each node
    %     - curr_vmStress:      (N_verts×1) vector of current von Mises stress at each node
    %     - meshTetrahedrons:   (N_tet×4) connectivity
    %     - MaxStress:          scalar (maximum allowable von Mises stress)
    %     - alpha:              scalar (barrier‐function tuning constant)
    %     - Kcbf:               scalar (weight on \|u−u_des\|^2)
    %     - Ce, Le, J:          FEM ingredients for d(σ_vM)/du
    %
    %   Outputs:
    %     - AgentsVels:         (3N_agents×1) the QP‐corrected velocity
    %     - A, b:               matrices so that A*u <= b enforces barrier
    %     - grad_h:             (N_verts×3N_agents) d(σ_vM)/du per node
    %     - scaled_delta:       (6×N_verts) partial d(σ_vM)/d(σ_tensor)
    %     - pred_nodal_stresses: (6*N_verts × 3N_agents) predicted nodal stresses
    %     - pred_stresses_element, pred_strains, element_displacements: intermediates
    %

    % -------------------------
    % 1) Form h = MaxStress - current von Mises
    % -------------------------
    h = MaxStress - curr_vmStress;   % (N_verts×1)

    % -------------------------
    % 2) Compute ∇h (size: N_verts × 3N_agents)
    % -------------------------
    [grad_h, scaled_delta, pred_nodal_stresses, pred_stresses_element, pred_strains, element_displacements] = computeVonMisesDerivative_Debug(curr_StressTensor, curr_vmStress, Ce, Le, J, meshTetrahedrons);

    % -------------------------
    % 3) Build QP: minimize  (u - u_des)'*(u - u_des),
    %    which is  u'*(2I)*u  +  (-2 u_des)'*u  +  constant.
    %    So H = 2*Kcbf*I,   f = -2*Kcbf*u_des.
    % -------------------------
    nU = length(AgentActions);               % should equal 3*N_agents
    H  = 2 * Kcbf * eye(nU);
    f  = -2 * Kcbf * AgentActions;            % push “u” toward the original AgentActions (u_des)

    % -------------------------
    % 4) Build inequality -∇h * u <= α * h
    % -------------------------
    A = -grad_h;      % (N_verts×3N_agents)
    b =  alpha * h;   % (N_verts×1)

    % tStart = tic;
    % -------------------------
    % 5) Solve the QP
    % -------------------------
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    [AgentsVels, ~, exitflag] = quadprog(H, f, A, b, [], [], [], [], [], opts);

    % tEnd = toc(tStart);
    % fprintf("Computation completed after %f seconds.\n", tEnd);

    if exitflag ~= 1
        warning("No feasible solution found for barrier QP; returning zero velocities.");
        AgentsVels = zeros(nU,1);
    end



    % Verification of the condition
    [row, col, res] = find(h < 0);
    if res ~= 0
        disp("Current velocities computed = ");
        disp(AgentsVels);
        for i = 1:length(res)
            disp(strcat("currStress in id ", num2str(row(i)), " = ", num2str(curr_vmStress(row(i),col(i)))));
        end
        % error('Not allowed stress');
    end
end


%% Inverse computation (NOT Including the control action)
function [grad_h, scaled_delta, pred_nodal_stresses, pred_stresses_element, pred_strains, element_displacements] = computeVonMisesDerivative_Debug(curr_stress, VM_stress, Ce, Le, J, meshTetrahedrons)

    % computeVonMisesDerivative_Debug
    %   Numerically builds the mapping ∇_u(σ_vM) by:
    %     1) computing ∆ = d(σ_vM)/d(σ_tensor) at each node
    %     2) mapping nodal displacements → element displacements via G
    %     3) running the usual FEM chain to get “predicted” element stresses
    %     4) assembling them back onto nodes
    %     5) doing the chain‐rule sum for each node to obtain ∇u (size: N_verts×3N_agents)
    %
    %   Inputs:
    %     - curr_stress:   (N_verts×6) current stress tensor at each node
    %     - VM_stress:     (N_verts×1) current von Mises at each node
    %     - Ce:            (6×6) elasticity matrix
    %     - Le:            (6×12×N_tet) strain‐displacement blocks
    %     - J:             (3*N_verts×3*N_agents) global displacement for each agent
    %     - meshTetrahedrons: (N_tet×4)
    %
    %   Outputs:
    %     - grad_h:             (N_verts × 3N_agents)
    %     - scaled_delta:       (6×N_verts)  (∂σ_vM/∂σ) per node
    %     - pred_nodal_stresses: (6*N_verts × 3N_agents)
    %     - pred_stresses_element: (6*N_tet × 3N_agents)
    %     - pred_strains:         (6*N_tet × 3N_agents)
    %     - element_displacements: (12*N_tet × 3N_agents)
    %

    N_tet  = size(meshTetrahedrons, 1);
    N_verts = size(curr_stress, 1);
    N_a    = size(J, 2) / 3;   % number of agents

    %% 1) Compute d(σ_vM)/d(σ) “∆” for each node
    %    If σ = [σ11 σ22 σ33 σ12 σ13 σ23]ᵀ at a node,
    %    then
    %      σ_vM = sqrt(0.5*((σ11−σ22)^2 + (σ22−σ33)^2 + (σ33−σ11)^2) + 3*(σ12^2+σ13^2+σ23^2))
    %    So for a single node i:
    %
    %       denom_i = 2 * VM_stress(i) + eps
    %       ∂σ_vM/∂σ11 = (2σ11 - σ22 - σ33)/denom_i
    %       ∂σ_vM/∂σ22 = (-σ11 + 2σ22 - σ33)/denom_i
    %       ∂σ_vM/∂σ33 = (-σ11 - σ22 + 2σ33)/denom_i
    %       ∂σ_vM/∂σ12 = 6 σ12    /denom_i
    %       ∂σ_vM/∂σ13 = 6 σ13    /denom_i
    %       ∂σ_vM/∂σ23 = 6 σ23    /denom_i
    %
    scaled_delta = zeros(6, N_verts);
    sigma11 = curr_stress(:,1);
    sigma22 = curr_stress(:,2);
    sigma33 = curr_stress(:,3);
    sigma12 = curr_stress(:,4);
    sigma13 = curr_stress(:,5);
    sigma23 = curr_stress(:,6);
    den = 2 * VM_stress + eps;   % (N_verts×1)

    scaled_delta(1,:) = (2*sigma11 - sigma22 - sigma33)' ./ den';  % 1×N_verts
    scaled_delta(2,:) = (-sigma11 + 2*sigma22 - sigma33)' ./ den';
    scaled_delta(3,:) = (-sigma11 - sigma22 + 2*sigma33)' ./ den';
    scaled_delta(4,:) = (6*sigma12)' ./ den';
    scaled_delta(5,:) = (6*sigma13)' ./ den';
    scaled_delta(6,:) = (6*sigma23)' ./ den';
    % Now scaled_delta is 6×N_verts.

    %% 2) Build the “G” matrix to map nodal-u → element‐u
    elements = meshTetrahedrons';            % (4 × N_tet)
    elements = elements(:);                  % (4*N_tet × 1)
    nodes_rep = repelem(elements, 3);        % (12*N_tet × 1)
    comps    = repmat([1;2;3], 4*N_tet, 1);   % (12*N_tet × 1)
    colsG    = 3*(nodes_rep - 1) + comps;    % (12*N_tet × 1)
    rowsG    = (1 : 12*N_tet)';               % (12*N_tet × 1)
    G = sparse(rowsG, colsG, 1, 12*N_tet, 3*N_verts);

    % element_displacements is (12*N_tet × 3N_agents)
    element_displacements = G * J;  

    %% 3) Build the block‐diagonal Le to go from element‐u → element‐strain
    Le_cells = squeeze(num2cell(Le, [1,2]));   % 1×1×N_tet cell, each is 6×12
    Le_blck  = sparse(blkdiag(Le_cells{:}));    % (6*N_tet × 12*N_tet)

    % pred_strains: (6*N_tet × 3N_agents)
    pred_strains = Le_blck * element_displacements;

    %% 4) Build block‐diag Ce to go from element‐strain → element‐stress
    Ce_blck = kron(speye(N_tet), Ce);         % (6*N_tet × 6*N_tet)
    pred_stresses_element = Ce_blck * pred_strains; % (6*N_tet × 3N_agents)

    %% 5) Assemble element stresses back to nodal stresses
    %   First build the  (N_verts × N_tet) incidence:
    rowsW = meshTetrahedrons';     % 4 × N_tet
    rowsW = rowsW(:);              % (4*N_tet × 1)
    colsW = repelem(1:N_tet, 4)';   % (4*N_tet × 1)
    W_basic = sparse(rowsW, colsW, 1, N_verts, N_tet);   % (N_verts×N_tet)

    W_blck = kron(W_basic, speye(6));   % (6*N_verts × 6*N_tet)
    pred_nodal_stresses = W_blck * pred_stresses_element; % (6*N_verts × 3N_agents)

    denom_node = sum(W_blck, 2);        % (6*N_verts × 1)
    pred_nodal_stresses = pred_nodal_stresses ./ (denom_node + eps);

    %% 6) Finally, do the dot‐product along each node to collapse:
    %   For each agent‐column c = 1..(3N_agents), we have pred_nodal_stresses(:,c) is 6*N_verts × 1,
    %   but we only need to pick out the 6 components at node i, multiply by scaled_delta(:,i).
    %
    %   A straightforward way is to reshape “pred_nodal_stresses” into a 6×N_verts×(3N_agents) block,
    %   then do the weighted sum:
    P3 = reshape(pred_nodal_stresses, [6, N_verts, 3*N_a]);  % (6 × N_verts × 3N_agents)

    % preallocate grad_h
    grad_h = zeros(N_verts, 3*N_a);

    for c = 1:(3*N_a)
        % Δ at each node is scaled_delta(:, i), P3(:, i, c) is the 6 stresses
        % dot‐product for node i:
        % grad_h(i, c) = ∆(:,i)' * P3(:,i,c).  (That gives a scalar per node/agent pair.)
        grad_h(:,c) = squeeze(sum(scaled_delta .* P3(:,:,c), 1))';  % 1×N_verts → transpose to N_verts
    end

    % Now grad_h is (N_verts × 3N_agents).
end

