function [AgentsVels] = stressControlWithBarrier(curr_VertexPose, prev_VertexPose, ...
                            AgentsPose, AgentGoals, curr_StressTensor, curr_vmStress, ...
                            prev_StressTensor, YieldStress, SF, alpha, Katt, Krep)
% stressControlWithBarrier
% This function simulates a control system where agents move towards goals while avoiding high-stress regions.
%
% Inputs:
% 1. curr_VertexPose: Current positions of nodal points (n_vertices x ndim)
% 2. prev_VertexPose: Previous positions of nodal points (n_vertices x ndim)
% 3. AgentsPose: Current positions of agents (n_agents x ndim)
% 4. AgentGoals: Desired positions of agents (n_agents x ndim)
% 5. curr_StressTensor: Current stress values for each virtual point (n_vertices x 6)
% 6. curr_vmStress: Current von Mises stress values for each virtual point (n_vertices x 1)
% 7. prev_StressTensor: Previous stress values for each virtual point (n_vertices x 6)
% 8. Yield: Yield allowed stress threshold (scalar)
% 9. SF: Safety factor used to compute the Maximum allowed stress threshold (scalar)
% 9. alpha: Barrier function constraint parameter (scalar)
% 10. Katt: Attraction gain for agents towards goals (scalar)
% 11. Krep: Repulsion gain for virtual points away from stress (scalar)
%
% Outputs:
% 1. AgentVels: Computed velocity of the agents (n_agents x ndim)

    % Parameters
    n_vertices = size(curr_VertexPose, 1); % Number of virtual points
    n_agents = size(AgentsPose, 1);        % Number of agents
    ndim = size(AgentsPose, 2);           % Number of dimensions

    % Preallocate velocities
    AgentsVels = zeros(n_agents,ndim);
    v_virtual = zeros(n_vertices,ndim);

    % Set optimization options
    options = optimset('Display', 'off');

    % Compute the MaxStress
    MaxStress = YieldStress/SF;

    % Get time measurement
    tStart = tic; % Start time


    % Barrier function control for virtual points
    for i = 1:n_vertices
        % Compute stress barrier function
        h = MaxStress - curr_vmStress(i);
        grad_h = computeBarrierGradient(prev_StressTensor(i,:), curr_StressTensor(i,:), prev_VertexPose(i,:), curr_VertexPose(i,:), curr_vmStress(i));

        % Desired velocity for repulsion from stress
        vrep = Krep * (h / MaxStress);

        % Define QP problem
        H = 2 * eye(ndim); % Quadratic cost for velocity
        f = -2 * vrep';    % Linear term for desired velocity
        A = grad_h;        % Gradient of barrier function
        b = alpha * h;     % Barrier constraint

        % Solve QP for velocity
        v_virtual(i,:) = quadprog(H, f, A, b, [], [], [], [], [], options);

    end

    % Aggregate virtual point influence for agents
    for j = 1:n_agents
        % Compute agent's attraction velocity towards goal
        v_att = -Katt * (AgentsPose(j, :) - AgentGoals(j, :));

        % Aggregate virtual point repulsions based on proximity
        v_rep = zeros(1, ndim);
        for i = 1:n_vertices
            dist = norm(curr_VertexPose(i, :) - AgentsPose(j, :));
            if dist > 0
                weight = 1 / (dist^2); % Weight inversely proportional to squared distance
                v_rep = v_rep + weight * v_virtual(i,:);
            end
        end

        % Combine attraction and repulsion
        AgentsVels(j,:) = v_att + v_rep;
    end
    
    tEnd = toc(tStart);
    fprintf("Computation completed after %f seconds.\n", tEnd);
end

function grad_vm = computeVonMisesGradient(stress_tensor, sigma_vm)
    sigma_xx = stress_tensor(1, 1);
    sigma_yy = stress_tensor(2, 2);
    sigma_zz = stress_tensor(3, 3);
    sigma_xy = stress_tensor(1, 2);
    sigma_yz = stress_tensor(2, 3);
    sigma_zx = stress_tensor(3, 1);

    dA_dsigma_xx = (sigma_xx - sigma_yy) - (sigma_zz - sigma_xx);
    dA_dsigma_yy = (sigma_yy - sigma_zz) - (sigma_xx - sigma_yy);
    dA_dsigma_zz = (sigma_zz - sigma_xx) - (sigma_yy - sigma_zz);
    dA_dsigma_xy = 12 * sigma_xy;
    dA_dsigma_yz = 12 * sigma_yz;
    dA_dsigma_zx = 12 * sigma_zx;

    grad_A = [dA_dsigma_xx, dA_dsigma_yy, dA_dsigma_zz, dA_dsigma_xy, dA_dsigma_yz, dA_dsigma_zx];
    grad_vm = (1 / (2 * sigma_vm)) * grad_A;
end


function grad_stress_pos = computeStressGradient(prev_stress, curr_stress, prev_pos, curr_pos)
    % Inputs:
    % - prev_stress: [6x1] stress tensor at the previous state
    % - curr_stress: [6x1] stress tensor at the current state
    % - prev_pos: [1x3] position at the previous state
    % - curr_pos: [1x3] position at the current state

    % Compute finite difference approximation
    grad_stress_pos = (curr_stress - prev_stress) ./ (curr_pos - prev_pos);
end

function grad_h = computeBarrierGradient(prev_stress, curr_stress, prev_pos, curr_pos, sigma_vm)
    % Step 1: Gradient of von Mises w.r.t. stress tensor
    grad_vm = computeVonMisesGradient(curr_stress, sigma_vm);

    % Step 2: Gradient of stress tensor w.r.t. position
    grad_stress_pos = computeStressGradient(prev_stress, curr_stress, prev_pos, curr_pos);

    % Step 3: Combine gradients
    grad_h = -grad_vm * grad_stress_pos';
end
