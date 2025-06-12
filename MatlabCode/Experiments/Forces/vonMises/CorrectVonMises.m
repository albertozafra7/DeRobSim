%% Data generation
gm = multicuboid(0.06,0.005,0.01);
% figure
% pdegplot(gm,FaceLabels="on",FaceAlpha=0.5)
% view(50,20)

% Create an femodel object for solving a static structural problem, and assign the geometry to the model.
model = femodel(AnalysisType="structuralStatic", ...
                Geometry=gm);
% Specify Young's modulus, Poisson's ratio, and the mass density of the material.
model.MaterialProperties = materialProperties(YoungsModulus=210e3, ...
                                              PoissonsRatio=0.3, ...
                                              MassDensity=2.7e-6);

% Specify a gravity load on the beam.
model.CellLoad = cellLoad(Gravity=[0;0;-9.8]);

% Fix one end of the beam.
model.FaceBC(5) = faceBC(Constraint="fixed");

% Generate a mesh.
model = generateMesh(model,GeometricOrder="linear");

R = solve(model);

%% PDE - Von Mises
% Evaluate the von Mises stress in the beam.
% vmStress = evaluateVonMisesStress(R);

% Plot the von Mises stress for the last time-step.
figure
pdeviz(R.Mesh,R.VonMisesStress)
title("von Mises Stress in the Beam for the Last Time-Step")

%% Correct von Mises
n_nodes = length(R.Mesh.Nodes);
% n_times = length(tlist);

E = 210e3; % Young Modulus (Pa)
nu = 0.3; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)

VertexInitPoses = R.Mesh.Nodes'; 
VertexFinalPoses = zeros(n_nodes,3);

VertexFinalPoses(:,1) = VertexInitPoses(:,1) + R.Displacement.x(:);
VertexFinalPoses(:,2) = VertexInitPoses(:,2) + R.Displacement.y(:);
VertexFinalPoses(:,3) = VertexInitPoses(:,3) + R.Displacement.z(:);

VertexInterPoses = zeros(n_nodes,3);

VertexInterPoses(:,1) = VertexInitPoses(:,1);
VertexInterPoses(:,2) = VertexInitPoses(:,2);
VertexInterPoses(:,3) = VertexInitPoses(:,3);

MeshTetrahedrons = model.Geometry.Mesh.Elements';
% MeshTetrahedrons = sort(MeshTetrahedrons,2,"ascend");
% SubMeshTetrahedrons = MeshTetrahedrons(:,2:end);
% SubMeshTetrahedrons = sort(SubMeshTetrahedrons,2,"ascend");
% MeshTetrahedrons(:,2:end) = SubMeshTetrahedrons;

volumes = abs(dot(cross(VertexInitPoses(MeshTetrahedrons(:,2),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), ...
                        VertexInitPoses(MeshTetrahedrons(:,3),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), 2), ...
                        VertexInitPoses(MeshTetrahedrons(:,4),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), 2)) / 6;

% Compute element stiffness matrix (6x6xN_tet)
Ce = CeMatrixComputation(E, nu);

% Compute strain-displacement matrices (6x12xN_tet)
Le = ComputeBe(MeshTetrahedrons, VertexInitPoses);
% 
% % Initialize displacement vector (12x1xN_tet)
% u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));
% 
% % Construct displacement vectors for all tetrahedrons
% for elem = 1:size(MeshTetrahedrons, 1)
%     nodes = MeshTetrahedrons(elem, :); % Get the global node indices
%     for node = 1:4
%         u_hat_e(3*node-2,1,elem) = R.Displacement.x(nodes(node)); % x-displacement
%         u_hat_e(3*node-1,1,elem) = R.Displacement.y(nodes(node)); % y-displacement
%         u_hat_e(3*node,1,elem)   = R.Displacement.z(nodes(node)); % z-displacement
%     end
% end
% 
% % Compute strains (6x1xN_tet)
% strains_disp = pagemtimes(Le, u_hat_e);
% 
% % Compute stresses (6x1xN_tet)
% stresses_disp = pagemtimes(Ce, strains_disp);
% 
% % Initialize node-based stress tensor
% stress_n_disp = zeros(6,1,n_nodes);
% node_contributions = zeros(n_nodes, 1); % Track contributions per node
% 
% % Accumulate stresses for each node
% for elem = 1:size(MeshTetrahedrons, 1)
%     nodes = MeshTetrahedrons(elem, :);
%     stress_elem = stresses_disp(:,:,elem);
%     for i = 1:4
%         stress_n_disp(:,:,nodes(i)) = stress_n_disp(:,:,nodes(i)) + stress_elem;
%         node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
%     end
% end
% 
% % Average stress at each node
% for node = 1:n_nodes
%     if node_contributions(node) > 0
%         stress_n_disp(:,:,node) = stress_n_disp(:,:,node) / node_contributions(node);
%     end
% end
[~, stress_n_disp, ~, ~,~] = ComputeStressByDisplacements([R.Displacement.x'; R.Displacement.y'; R.Displacement.z']',MeshTetrahedrons, Ce, Le);

% Compute von Mises stress at each node
VMSigma_n_Disp = zeros(n_nodes, 1);
for node = 1:n_nodes
    sigma_12 = stress_n_disp(1,node) - stress_n_disp(2,node); % sigma_x - sigma_y
    sigma_23 = stress_n_disp(2,node) - stress_n_disp(3,node); % sigma_y - sigma_z
    sigma_31 = stress_n_disp(3,node) - stress_n_disp(1,node); % sigma_z - sigma_x
    VMSigma_n_Disp(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
        3 * (stress_n_disp(4,node)^2 + stress_n_disp(5,node)^2 + stress_n_disp(6,node)^2));
end


% Plot von Mises stress
figure;
pdeviz(R.Mesh, VMSigma_n_Disp);
title("von Mises Stress Displacements");

% Compare with result stress and compute relative error
Result_Stress = [R.Stress.sxx, R.Stress.syy, R.Stress.szz, R.Stress.sxy, R.Stress.syz, R.Stress.szx];
stressError = abs(squeeze(stress_n_disp)' - Result_Stress);% ./ max(Result_Stress, [], 'all');
disp("Maximum Absolute Stress Error - With Displacements:");
disp(max(stressError));

% Compare with result von mises stress and compute relative error
VMError = abs(VMSigma_n_Disp - R.VonMisesStress) ./ R.VonMisesStress;
disp(strcat("Maximum Relative von Mises Error - With Displacements: ", num2str(max(VMError))));
disp(strcat("Maximum Porcentual von Mises Error - With Displacements:",num2str(max(VMError) * 100), "%"));


%% von Mises With our data

% Initialize displacement vector (12x1xN_tet)
u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));

% Construct displacement vectors for all tetrahedrons
for elem = 1:size(MeshTetrahedrons, 1)
    nodes = MeshTetrahedrons(elem, :); % Get the global node indices
    for node = 1:4
        u_hat_e(3*node-2,1,elem) = VertexFinalPoses(nodes(node),1) - VertexInitPoses(nodes(node),1); % x-displacement
        u_hat_e(3*node-1,1,elem) = VertexFinalPoses(nodes(node),2) - VertexInitPoses(nodes(node),2); % y-displacement
        u_hat_e(3*node,1,elem)   = VertexFinalPoses(nodes(node),3) - VertexInitPoses(nodes(node),3); % z-displacement
    end
end

% Compute strains (6x1xN_tet)
strains_disp = pagemtimes(Le, u_hat_e);

% Compute stresses (6x1xN_tet)
stresses_disp = pagemtimes(Ce, strains_disp);

% Initialize node-based stress tensor
stress_n_disp = zeros(6,1,n_nodes);
node_contributions = zeros(n_nodes, 1); % Track contributions per node

% Accumulate stresses for each node
for elem = 1:size(MeshTetrahedrons, 1)
    nodes = MeshTetrahedrons(elem, :);
    stress_elem = stresses_disp(:,:,elem);
    for i = 1:4
        stress_n_disp(:,:,nodes(i)) = stress_n_disp(:,:,nodes(i)) + stress_elem;
        node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
    end
end

% Average stress at each node
for node = 1:n_nodes
    if node_contributions(node) > 0
        stress_n_disp(:,:,node) = stress_n_disp(:,:,node) / node_contributions(node);
    end
end

% Compute von Mises stress at each node
VMSigma_n_Disp = zeros(n_nodes, 1);
for node = 1:n_nodes
    sigma_12 = stress_n_disp(1,:,node) - stress_n_disp(2,:,node); % sigma_x - sigma_y
    sigma_23 = stress_n_disp(2,:,node) - stress_n_disp(3,:,node); % sigma_y - sigma_z
    sigma_31 = stress_n_disp(3,:,node) - stress_n_disp(1,:,node); % sigma_z - sigma_x
    VMSigma_n_Disp(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
        3 * (stress_n_disp(4,:,node)^2 + stress_n_disp(5,:,node)^2 + stress_n_disp(6,:,node)^2));
end


% Plot von Mises stress
figure;
pdeviz(R.Mesh, VMSigma_n_Disp);
title("von Mises Stress Custom Data");

% Compare with result stress and compute relative error
Result_Stress = [R.Stress.sxx, R.Stress.syy, R.Stress.szz, R.Stress.sxy, R.Stress.syz, R.Stress.szx];
stressError = abs(squeeze(stress_n_disp)' - Result_Stress);% ./ max(Result_Stress, [], 'all');
disp("Maximum Absolute Stress Error - Custom Data:");
disp(max(stressError));

% Compare with result von mises stress and compute relative error
VMError = abs(VMSigma_n_Disp - R.VonMisesStress) ./ R.VonMisesStress;
disp(strcat("Maximum Relative von Mises Error - Custom Data: ", num2str(max(VMError))));
disp(strcat("Maximum Porcentual von Mises Error - Custom Data:",num2str(max(VMError) * 100), "%"));







%-------------------------------------------------------------------


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




function B = ComputeBe(MeshTetraedrons, VertexPose)
    N_tet = length(MeshTetraedrons);
    B = zeros(6,12,N_tet);
    for e = 1:N_tet
        % Extract indices of the four vertices for the current tetrahedron
        indices = MeshTetraedrons(e, :);
        P1 = VertexPose(indices(1), :); % First point
        P2 = VertexPose(indices(2), :); % Second point
        P3 = VertexPose(indices(3), :); % Third point
        P4 = VertexPose(indices(4), :); % Fourth point

        grads = computeShapeFunctionGradients(P1, P2, P3, P4);
        % V = abs(det([1, P1; 1, P2; 1, P3; 1, P4])) / 6;

        for i = 1:4
            % Each column block corresponds to one vertex
            B(1, 3*i-2, e) = grads(i ,1); % dN_i/dx
            B(2, 3*i-1, e) = grads(i, 2); % dN_i/dy
            B(3, 3*i, e)   = grads(i, 3); % dN_i/dz
            B(4, 3*i-2, e) = grads(i, 2); % dN_i/dy
            B(4, 3*i-1, e) = grads(i, 1); % dN_i/dx
            B(5, 3*i-1, e) = grads(i, 3); % dN_i/dz
            B(5, 3*i, e)   = grads(i, 2); % dN_i/dy
            B(6, 3*i-2, e) = grads(i, 3); % dN_i/dz
            B(6, 3*i, e)   = grads(i, 1); % dN_i/dx
        end
    end
end

function grads = computeShapeFunctionGradients(P1, P2, P3, P4)
    % Computes the gradients of the shape functions for a tetrahedron
    % Inputs:
    % P1, P2, P3, P4: Coordinates of the tetrahedron vertices as 1x3 arrays
    % Output:
    % grads: 3x4 matrix of shape function gradients

    % Build the Jacobian matrix (3x3 matrix)
    J = [P2 - P1;  % Vector P1 -> P2
         P3 - P1;  % Vector P1 -> P3
         P4 - P1]'; % Vector P1 -> P4

    % Compute the determinant of the Jacobian to ensure it is non-zero
    detJ = det(J);
    if abs(detJ) < 1e-12
        error('Jacobian is singular or nearly singular. Check input points.');
    end

    % Compute the inverse of the Jacobian matrix
    invJ = inv(J);

    % Gradients of shape functions in local coordinates (reference tetrahedron)
    % Reference shape function gradients with respect to local coords (ξ, η, ζ)
    local_grads = [-1, -1, -1;  % ∂N1/∂ξ, ∂N1/∂η, ∂N1/∂ζ
                    1,  0,  0;  % ∂N2/∂ξ, ∂N2/∂η, ∂N2/∂ζ
                    0,  1,  0;  % ∂N3/∂ξ, ∂N3/∂η, ∂N3/∂ζ
                    0,  0,  1]; % ∂N4/∂ξ, ∂N4/∂η, ∂N4/∂ζ (4x3)

    % Transform local gradients to global coordinates using the Jacobian inverse
    grads = (invJ' * local_grads')'; % 3x4 matrix
end