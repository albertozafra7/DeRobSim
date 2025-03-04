%% Plotting configuration
groundtruth = true;

%% Data generation
gm = multicuboid(0.06, 0.005, 0.01);
model = femodel(AnalysisType="structuralStatic", Geometry=gm);
model.MaterialProperties = materialProperties(YoungsModulus=210e3, PoissonsRatio=0.3, MassDensity=2.7e-6);
model.CellLoad = cellLoad(Gravity=[0; 0; -9.8]);
model.FaceBC(5) = faceBC(Constraint="fixed");
model = generateMesh(model, GeometricOrder="linear");
R = solve(model);

%% Compute Approximate Alpha Value
% Use mean edge length of the mesh for alpha value estimation
nodes = R.Mesh.Nodes';
edges = nchoosek(1:size(nodes, 1), 2); % Pairs of points
distances = sqrt(sum((nodes(edges(:, 1), :) - nodes(edges(:, 2), :)).^2, 2)); % Edge lengths
alphaValue = 1.5 * mean(distances); % Scale mean edge length for alpha shape

%% Extract Surface Mesh Using Alpha Shape
% Create alpha shape and extract surface triangulation
shp = alphaShape(nodes(:, 1), nodes(:, 2), nodes(:, 3), alphaValue);

% Extract triangulation from alpha shape
[T, X] = boundaryFacets(shp); % T: triangles, X: vertices
MeshTriangles = T; % Each column is a triangle defined by 3 nodes
VertexInitPoses = X; % Node coordinates (3xN matrix)

n_verts = size(VertexInitPoses, 1); % Number of nodes
n_triang = size(MeshTriangles, 1); % Number of triangular elements

% Map displacements from original mesh to alpha shape nodes
nodeCoords = R.Mesh.Nodes'; % N x 3 matrix, where N is the number of nodes

% Ensure displacements are column vectors
displacementsX = R.Displacement.x(:); % Convert to N x 1 column vector
displacementsY = R.Displacement.y(:); % Convert to N x 1 column vector
displacementsZ = R.Displacement.z(:); % Convert to N x 1 column vector

% Create interpolants for each displacement component
interpDispX = scatteredInterpolant(nodeCoords(:, 1), nodeCoords(:, 2), nodeCoords(:, 3), displacementsX);
interpDispY = scatteredInterpolant(nodeCoords(:, 1), nodeCoords(:, 2), nodeCoords(:, 3), displacementsY);
interpDispZ = scatteredInterpolant(nodeCoords(:, 1), nodeCoords(:, 2), nodeCoords(:, 3), displacementsZ);

% Interpolate displacements at the surface nodes
interpDisplacementsX = interpDispX(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));
interpDisplacementsY = interpDispY(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));
interpDisplacementsZ = interpDispZ(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));

% Combine interpolated displacements into a single matrix
interpDisplacements = [interpDisplacementsX, interpDisplacementsY, interpDisplacementsZ];

% We generate the final positions
VertexFinalPoses = VertexInitPoses + interpDisplacements;

vonMisesStressNodes = computeVonMisesStress(VertexInitPoses,MeshTriangles,interpDisplacements);


%% Compare with MATLAB Built-In von Mises Stress
% Evaluate von Mises stress using MATLAB's built-in function
vmStress_builtin = R.VonMisesStress;

% Interpolate built-in von Mises stress at surface nodes
interpVMStress_builtin = scatteredInterpolant( ...
    R.Mesh.Nodes(1, :)', R.Mesh.Nodes(2, :)', R.Mesh.Nodes(3, :)', vmStress_builtin);
vmStress_builtin_surface = interpVMStress_builtin(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));

%% Stress Per Node
real_stresses = zeros(6,length(R.Stress.sxx));
real_stresses(1,:) = R.Stress.sxx(:);
real_stresses(2,:) = R.Stress.syy(:);
real_stresses(3,:) = R.Stress.szz(:);
real_stresses(4,:) = R.Stress.sxy(:);
real_stresses(5,:) = R.Stress.sxz(:);
real_stresses(6,:) = R.Stress.syz(:);

interpStress_builtin =scatteredInterpolant( ...
    R.Mesh.Nodes(1, :)', R.Mesh.Nodes(2, :)', R.Mesh.Nodes(3, :)', real_stresses');
Stress_builtin_surface = interpStress_builtin(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));

% for vert = 1:n_verts
%     vonMisesStressNodes(vert) = ComputeVonMises(Stress_builtin_surface(vert,:));
% end

%% Strain Per Node
real_strains = zeros(6,length(R.Strain.xx));
real_strains(1,:) = R.Strain.xx(:);
real_strains(2,:) = R.Strain.yy(:);
real_strains(3,:) = R.Strain.zz(:);
real_strains(4,:) = R.Strain.xy(:);
real_strains(5,:) = R.Strain.xz(:);
real_strains(6,:) = R.Strain.yz(:);

interpStrain_builtin =scatteredInterpolant( ...
    R.Mesh.Nodes(1, :)', R.Mesh.Nodes(2, :)', R.Mesh.Nodes(3, :)', real_strains');
Strain_builtin_surface = interpStrain_builtin(VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3));


%% Errors

% % Stresses
% relativeStressError = abs(StressNodes - Stress_builtin_surface') ./ max(Stress_builtin_surface');
% maxStressError = max(relativeStressError');
% disp("Maximum Stress Relative Error: ");
% disp(maxStressError);
% 
% % Strains
% relativeStrainError = abs(StrainNodes - Strain_builtin_surface') ./ max(Strain_builtin_surface');
% maxStrainError = max(relativeStrainError');
% disp("Maximum Strain Relative Error: ");
% disp(maxStrainError);


% Von Mises
relativeError = abs(vonMisesStressNodes' - vmStress_builtin_surface') ./ max(vmStress_builtin_surface');
maxError = max(relativeError);
disp("Maximum von Mises Relative Error: " + maxError);

%% Plot Results

if groundtruth

    % Ensure both stress arrays are column vectors
    vonMisesStressNodes = vonMisesStressNodes(:);
    vmStress_builtin_surface = vmStress_builtin_surface(:);
    
    % Normalize color scales for consistent comparison
    minStress = min(R.VonMisesStress); % Concatenate as column vectors
    maxStress = max(R.VonMisesStress);
    
    % Plot custom von Mises stress computation
    figure;
    
    subplot(1, 3, 1);
    trisurf(MeshTriangles, ...
        VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3), ...
        vonMisesStressNodes, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("Planar Stress Combination");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Plot MATLAB's built-in von Mises stress interpolated to surface nodes
    subplot(1, 3, 2);
    trisurf(MeshTriangles, ...
        VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3), ...
        vmStress_builtin_surface, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("3D Stress Interpolation");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Builtin Matlab von Mises stress
    subplot(1, 3, 3);
    pdeplot3D(R.Mesh, 'ColorMapData', R.VonMisesStress, 'FaceAlpha', 1);
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    colormap(parula); % Apply colormap to this axes
    title("GroundTruth");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
else
    % Ensure both stress arrays are column vectors
    vonMisesStressNodes = vonMisesStressNodes(:);
    vmStress_builtin_surface = vmStress_builtin_surface(:);
    
    % Normalize color scales for consistent comparison
    minStress = min(vmStress_builtin_surface); % Concatenate as column vectors
    maxStress = max(vmStress_builtin_surface);
    
    % Plot custom von Mises stress computation
    figure;
    
    subplot(1, 2, 1);
    trisurf(MeshTriangles, ...
        VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3), ...
        vonMisesStressNodes, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("Planar Stress Combination");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Plot MATLAB's built-in von Mises stress interpolated to surface nodes
    subplot(1, 2, 2);
    trisurf(MeshTriangles, ...
        VertexInitPoses(:, 1), VertexInitPoses(:, 2), VertexInitPoses(:, 3), ...
        vmStress_builtin_surface, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("3D Stress Interpolation");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
end

%% Custom Von MisesStress
% Compute 3D von Mises stress from a triangular surface mesh
% Input:
% - nodes: Nx3 matrix containing the 3D coordinates of each node.
% - elements: Mx3 matrix containing the node indices of each triangular element.
% - displacements: Nx3 matrix containing the nodal displacements (ux, uy, uz).

function vonMisesStress = computeVonMisesStress(nodes, elements, displacements)
    % Number of nodes and elements
    numNodes = size(nodes, 1);
    numElements = size(elements, 1);

    % Initialize stress tensors at each node
    % stressXYTensor = zeros(numNodes, 3); % [σxx, σyy, τxy]
    % stressXZTensor = zeros(numNodes, 3); % [σxx, σzz, τxz]
    % stressYZTensor = zeros(numNodes, 3); % [σyy, σzz, τyz]
    stressTensor = zeros(numNodes, 6);
    
    % Initialize the node contributions
    nodeContributions = zeros(numNodes, 1);

    % Loop over each triangular element
    for e = 1:numElements
        % Extract node indices for the current element
        elementNodes = elements(e, :);
        
        % Extract coordinates and displacements of the nodes
        coords = nodes(elementNodes, :);
        disp = displacements(elementNodes, :);
        
        % Compute the strain in each plane (xy, xz, yz)
        % Use finite element methods here
        [strainXY, strainXZ, strainYZ] = computePlaneStrains(coords, disp);

        % We correct the Nan elements
        strainXY(isnan(strainXY)) = 0;
        strainXZ(isnan(strainXZ)) = 0;
        strainYZ(isnan(strainYZ)) = 0;

        % Convert strains to stresses (plane stress assumption)
        [stressXY, stressXZ, stressYZ] = strainsToStresses(strainXY, strainXZ, strainYZ);

        % We correct the Nan elements
        stressXY(isnan(stressXY)) = 0;
        stressXZ(isnan(stressXZ)) = 0;
        stressYZ(isnan(stressYZ)) = 0;

        % Combine the normal stresses (averaging them where necessary)
        sigma_xx = sqrt(stressXY(1)^2 + stressXZ(1)^2);
        sigma_yy = sqrt(stressXY(2)^2 + stressYZ(1)^2);
        sigma_zz = sqrt(stressXZ(2)^2 + stressYZ(2)^2);

        % Extract shear stresses directly
        sigma_xy = stressXY(3);  % Shear stress in the xy-plane
        sigma_xz = stressXZ(3);  % Shear stress in the xz-plane
        sigma_yz = stressYZ(3);  % Shear stress in the yz-plane
        
        % Combine into a 6x1 stress vector
        stressVector = [
            sigma_xx;  % sigma_xx
            sigma_yy;  % sigma_yy
            sigma_zz;  % sigma_zz
            sigma_xy;  % sigma_xy
            sigma_xz;  % sigma_xz
            sigma_yz   % sigma_yz
        ];

        for n = 1:3
            node = elementNodes(n);
            
            % We accumulate the node contributions
            nodeContributions(node) = nodeContributions(node) + 1;
            % We accumulate the stress tensors
            % stressXYTensor(node,:) = stressXYTensor(node,:) + stressXY;
            % stressXZTensor(node,:) = stressXZTensor(node,:) + stressXZ;
            % stressYZTensor(node,:) = stressYZTensor(node,:) + stressYZ;
            stressTensor(node, :) = stressTensor(node, :) + stressVector';
            
        end

    end

    % stressXYTensor = stressXYTensor ./ nodeContributions;
    % stressXZTensor = stressXZTensor ./ nodeContributions;
    % stressYZTensor = stressYZTensor ./ nodeContributions;
    stressTensor = stressTensor ./ nodeContributions;


    % Combine stresses to form the stress tensor
    % vm_xy = sqrt(stressXYTensor(:, 1).^2 - stressXYTensor(:, 1) .* stressXYTensor(:, 2) + stressXYTensor(:, 2).^2 + 3 .* stressXYTensor(:, 3).^2);
    % vm_xz = sqrt(stressXZTensor(:, 1).^2 - stressXZTensor(:, 1) .* stressXZTensor(:, 2) + stressXZTensor(:, 2).^2 + 3 .* stressXZTensor(:, 3).^2);
    % vm_yz = sqrt(stressYZTensor(:, 1).^2 - stressYZTensor(:, 1) .* stressYZTensor(:, 2) + stressYZTensor(:, 2).^2 + 3 .* stressYZTensor(:, 3).^2);

    % Combine plane stresses for 3D von Mises approximation
    % vonMisesStress = sqrt(vm_xy.^2 + vm_xz.^2 + vm_yz.^2);
    vonMisesStress = zeros(numNodes,1);

    for node = 1:numNodes
        sigma_12 = stressTensor(node, 1) - stressTensor(node, 2); % sigma_x - sigma_y
        sigma_23 = stressTensor(node, 2) - stressTensor(node, 3); % sigma_y - sigma_z
        sigma_31 = stressTensor(node, 3) - stressTensor(node, 1); % sigma_z - sigma_x
        vonMisesStress(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
            3 * (stressTensor(node, 4)^2 + stressTensor(node, 5)^2 + stressTensor(node, 6)^2));
    end
end

function [strainXY, strainXZ, strainYZ] = computePlaneStrains(coords, node_disp)
    % Compute plane strains using the strain-displacement matrix and shape functions
    % Inputs:
    % - coords: 3x3 matrix of node coordinates for a triangular element
    % - disp: 3x3 matrix of node displacements for the same element
    % Outputs:
    % - strainXY, strainXZ, strainYZ: 3x3 matrices of strains for each plane

    % Precompute shape function derivatives in the natural coordinate system
    % For a triangle, assume linear shape functions: N1, N2, N3
    % dN/dxi and dN/deta are constants
    %
    % XY-plane
    Axy = abs(det([ones(3,1),coords(:,1), coords(:,2)]));
    % Axy = polyarea(coords(:,1), coords(:,2)); % Area of the triangle (xy-plane)
    Bxy = (1 / Axy) * [coords(2,2) - coords(3,2), 0, coords(3,2) - coords(1,2), 0, coords(1,2) - coords(2,2), 0;
                             0, coords(3,1) - coords(2,1), 0, coords(1,1) - coords(3,1), 0, coords(2,1) - coords(1,1);
                             coords(3,1) - coords(2,1), coords(2,2) - coords(3,2), coords(1,1) - coords(3,1), coords(3,2) - coords(1,2), coords(2,1) - coords(1,1), coords(1,2) - coords(2,2)];
    Uxy = [node_disp(:,1), node_disp(:,2)]';
    Uxy = Uxy(:);
    strainXY = Bxy * Uxy;

    % XZ-plane
    Axz = abs(det([ones(3,1),coords(:,1), coords(:,3)]));
    % Axz = polyarea(coords(:,1), coords(:,3)); % Area of the triangle (xz-plane)
    Bxz = (1 / Axz) * [coords(2,3) - coords(3,3), 0, coords(3,3) - coords(1,3), 0, coords(1,3) - coords(2,3), 0;
                             0, coords(3,1) - coords(2,1), 0, coords(1,1) - coords(3,1), 0, coords(2,1) - coords(1,1);
                             coords(3,1) - coords(2,1), coords(2,3) - coords(3,3), coords(1,1) - coords(3,1), coords(3,3) - coords(1,3), coords(2,1) - coords(1,1), coords(1,3) - coords(2,3)];
    Uxz = [node_disp(:,1), node_disp(:,3)]';
    Uxz = Uxz(:);
    strainXZ = Bxz * Uxz;

    % YZ-plane
    Ayz = abs(det([ones(3,1),coords(:,2), coords(:,3)]));
    % Ayz = polyarea(coords(:,2), coords(:,3)); % Area of the triangle (yz-plane)
    Byz = (1 / Ayz) * [coords(2,3) - coords(3,3), 0, coords(3,3) - coords(1,3), 0, coords(1,3) - coords(2,3), 0;
                             0, coords(3,2) - coords(2,2), 0, coords(1,2) - coords(3,2), 0, coords(2,2) - coords(1,2);
                             coords(3,2) - coords(2,2), coords(2,3) - coords(3,3), coords(1,2) - coords(3,2), coords(3,3) - coords(1,3), coords(2,2) - coords(1,2), coords(1,3) - coords(2,3)];
    Uyz = [node_disp(:,2), node_disp(:,3)]';
    Uyz = Uyz(:);
    strainYZ = Byz * Uyz;
end

function [stressXY, stressXZ, stressYZ] = strainsToStresses(strainXY, strainXZ, strainYZ)
    % Material properties (modify as needed)
    E = 210e3; % Young's modulus (Pa)
    nu = 0.3;  % Poisson's ratio

    % Plane stress constitutive matrix
    c11=E/(1-nu^2);
    c22=c11;
    c12=nu*c11;
    c21=c12;
    c33=E/(2*(1+nu));
    C = [c11, c12, 0; c21, c22, 0; 0, 0, c33];

    % Convert strains to stresses
    stressXY = (C * strainXY)';
    stressXZ = (C * strainXZ)';
    stressYZ = (C * strainYZ)';
end