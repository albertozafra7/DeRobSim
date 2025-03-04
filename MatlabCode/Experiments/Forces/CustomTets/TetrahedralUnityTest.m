% ############## Data Analysis Section ##############
%% Parameter Set
Time_id = -1; % 2;
% Plotting configuration
groundtruth = true; % Plots the comparison between methods and groundtruth
plotTrajectComp = false; % Plots the trajectory taken by the vertices/particles
plotShapeComp = false; % Plots the shape comparison between vertices and particles
% File and object names

% Tetrahedra
filename = "BeamSim_Tet";
filepath = "D:\alber\Documents\Cosas de la Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\Experiments\Forces\Unity\";
objectname = "DefBeam";


% Material Characteristics
E = 210e3; % Young Modulus
nu = 0.3;  % Poisson Ratio



%% Data Conversion
hdf52mat(strcat(filepath,filename,".h5"));
load(strcat(filepath,filename,".mat"));


%% Data Preparation
defObj = Unity(objectname);
particleInfo = defObj("particles_info");
vertexInfo = defObj("vertex_info");
tetrahedralInfo = defObj("tetverts_info");

n_times = length(times);
n_vertices = defObj("n_vertices");
n_vertices = n_vertices(1);
n_particles = defObj("n_particles");
n_particles = n_particles(1);
n_triangles = defObj("n_triangles");
n_triangles = n_triangles(1);
n_tet = defObj("n_tetrahedra");
n_tet = n_tet(1);
n_tet_verts = defObj("n_tetverts");
n_tet_verts = n_tet_verts(1);

% Preallocate arrays for each field
x_part = zeros(n_times, n_particles, 'single');
y_part = zeros(n_times, n_particles, 'single');
z_part = zeros(n_times, n_particles, 'single');

x_vert = zeros(n_times, n_vertices, 'single');
y_vert = zeros(n_times, n_vertices, 'single');
z_vert = zeros(n_times, n_vertices, 'single');

x_tet = zeros(n_times, n_tet_verts, 'single');
y_tet = zeros(n_times, n_tet_verts, 'single');
z_tet = zeros(n_times, n_tet_verts, 'single');

% Populate the arrays
for i = 1:n_times
    x_part(i, :) = particleInfo(i).x';
    y_part(i, :) = particleInfo(i).y';
    z_part(i, :) = particleInfo(i).z';

    x_vert(i, :) = vertexInfo(i).x';
    y_vert(i, :) = vertexInfo(i).y';
    z_vert(i, :) = vertexInfo(i).z';

    x_tet(i, :) = tetrahedralInfo(i).x';
    y_tet(i, :) = tetrahedralInfo(i).y';
    z_tet(i, :) = tetrahedralInfo(i).z';
end

% Triangles extraction
elements = zeros(n_triangles,3);
temp_triangles = defObj("mesh_triangles");

for e = 1:n_triangles
    elements(e,1) = temp_triangles(1).x(e) + 1;
    elements(e,2) = temp_triangles(1).y(e) + 1;
    elements(e,3) = temp_triangles(1).z(e) + 1;
end

% Tetrahedral extraction
tet_elements = zeros(n_tet,4);
temp_tet = defObj("mesh_tetrahedrons");

for e = 1:n_tet
    tet_elements(e,1) = temp_tet(1).x(e) + 1;
    tet_elements(e,2) = temp_tet(1).y(e) + 1;
    tet_elements(e,3) = temp_tet(1).z(e) + 1;
    tet_elements(e,4) = temp_tet(1).w(e) + 1;
end

if Time_id > n_times || Time_id < 1
    Time_id = n_times;
end

%% data Plotting
% Both Traject comp
if plotTrajectComp
    figure
    subplot(1, 2, 1);
    plot3(x_part(Time_id, :), z_part(Time_id, :), y_part(Time_id, :),"*");
    hold on
    plot3(x_part(1, :), z_part(1, :), y_part(1, :),"+");
    xlabel('X-axis');
    ylabel('Z-axis');
    zlabel('Y-axis');
    legend(["end", "start"]);
    title("Particles");
    
    subplot(1, 2, 2);
    plot3(x_vert(Time_id, :), z_vert(Time_id, :), y_vert(Time_id, :),"*");
    hold on
    plot3(x_vert(1, :), z_vert(1, :), y_vert(1, :),"+");
    xlabel('X-axis');
    ylabel('Z-axis');
    zlabel('Y-axis');
    legend(["end", "start"]);
    title("Vertices");
end

if plotShapeComp
    % Shape comp
    figure
    subplot(1, 2, 1);
    plot3(x_vert(1, :), z_vert(1, :), y_vert(1, :),"*");
    hold on
    plot3(x_part(1, :), z_part(1, :), y_part(1, :),"+");
    xlabel('X-axis');
    ylabel('Z-axis');
    zlabel('Y-axis');
    axis padded;
    title("Start");
    legend(["vertex", "particles"]);
    
    subplot(1, 2, 2);
    plot3(x_vert(Time_id, :), z_vert(Time_id, :), y_vert(Time_id, :),"*");
    hold on
    plot3(x_part(Time_id, :), z_part(Time_id, :), y_part(Time_id, :),"+");
    xlabel('X-axis');
    ylabel('Z-axis');
    zlabel('Y-axis');
    axis padded;
    title("End");
    legend(["vertex", "particles"]);
end    

%% ###### Force Test ######
particle_Initpose = zeros(n_particles,3);
particle_Initpose(:,1) = x_part(1,:);
particle_Initpose(:,2) = y_part(1,:);
particle_Initpose(:,3) = z_part(1,:);

particle_Finalpose = zeros(n_particles,3);
particle_Finalpose(:,1) = x_part(Time_id,:);
particle_Finalpose(:,2) = y_part(Time_id,:);
particle_Finalpose(:,3) = z_part(Time_id,:);


vertex_Initpose = zeros(n_vertices,3);
vertex_Initpose(:,1) = x_vert(1,:);
vertex_Initpose(:,2) = y_vert(1,:);
vertex_Initpose(:,3) = z_vert(1,:);

vertex_Finalpose = zeros(n_vertices,3);
vertex_Finalpose(:,1) = x_vert(Time_id,:);
vertex_Finalpose(:,2) = y_vert(Time_id,:);
vertex_Finalpose(:,3) = z_vert(Time_id,:);

vertex_Displacements = vertex_Finalpose - vertex_Initpose;

tetra_Initpose = zeros(n_tet_verts,3);
tetra_Initpose(:,1) = x_tet(1,:);
tetra_Initpose(:,2) = y_tet(1,:);
tetra_Initpose(:,3) = z_tet(1,:);

tetra_Finalpose = zeros(n_tet_verts,3);
tetra_Finalpose(:,1) = x_tet(Time_id,:);
tetra_Finalpose(:,2) = y_tet(Time_id,:);
tetra_Finalpose(:,3) = z_tet(Time_id,:);

%% Mesh generation
% We generate the FE Geometry
gm = fegeometry(vertex_Initpose, elements);
model = femodel(AnalysisType="structuralStatic", Geometry=gm);
model.MaterialProperties = materialProperties(YoungsModulus=E, PoissonsRatio=nu, MassDensity=2.7e-6);
model = generateMesh(model, GeometricOrder="linear");
mesh_model = model.Geometry.Mesh;

% We interpolate the displacements
% Extract refined mesh node positions
newNodes = model.Geometry.Mesh.Nodes'; % Transpose for [x, y, z] format
n_newNodes = size(newNodes, 1);

% Interpolate displacements to new nodes
interpX = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,1), 'linear', 'nearest');
interpY = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,2), 'linear', 'nearest');
interpZ = scatteredInterpolant(vertex_Initpose(:,1), vertex_Initpose(:,2), vertex_Initpose(:,3), vertex_Displacements(:,3), 'linear', 'nearest');

vertex_meshDisplacements = zeros(n_newNodes, 3);
vertex_meshDisplacements(:,1) = interpX(newNodes(:,1), newNodes(:,2), newNodes(:,3));
vertex_meshDisplacements(:,2) = interpY(newNodes(:,1), newNodes(:,2), newNodes(:,3));
vertex_meshDisplacements(:,3) = interpZ(newNodes(:,1), newNodes(:,2), newNodes(:,3));


%% Obtain the von Mises Stress by combining planar stresses

[vonMisesStressNodes, Stress] =  Compute3DvonMises(tetra_Initpose, tetra_Finalpose, tet_elements, E, nu);
vonMisesStressNodes(isnan(vonMisesStressNodes)) = 0;

%% Obtain the von Mises Stress with the tetrahedrons
[tet_VMSigma, tet_Stress] = Compute3DvonMises(mesh_model.Nodes', mesh_model.Nodes'+vertex_meshDisplacements, mesh_model.Elements', E, nu);


% Interpolate built-in von Mises stress at surface nodes
interpVMStress_builtin = scatteredInterpolant( ...
    mesh_model.Nodes(1, :)', mesh_model.Nodes(2, :)', mesh_model.Nodes(3, :)', tet_VMSigma);
vmStress_builtin_surface = interpVMStress_builtin(vertex_Initpose(:, 1), vertex_Initpose(:, 2), vertex_Initpose(:, 3));

%% Errors

% % Von Mises
% relativeError = abs(vonMisesStressNodes' - vmStress_builtin_surface') ./ max(vmStress_builtin_surface');
% maxError = max(relativeError);
% disp("Maximum von Mises Relative Error: " + maxError);

%% Plot Results
if groundtruth

    % Ensure both stress arrays are column vectors
    vonMisesStressNodes = vonMisesStressNodes(:);
    vmStress_builtin_surface = vmStress_builtin_surface(:);
    
    % Normalize color scales for consistent comparison
    minStress = min(tet_VMSigma); % Concatenate as column vectors
    maxStress = max(tet_VMSigma);
    
    % Plot custom von Mises stress computation
    figure;
    
    subplot(1, 3, 1);
    pdeplot3D(tetra_Initpose', tet_elements', 'ColorMapData', vonMisesStressNodes, 'FaceAlpha', 1);
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    colormap(parula); % Apply colormap to this axes
    title("TetSim Stress");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Plot MATLAB's built-in von Mises stress interpolated to surface nodes
    subplot(1, 3, 2);
    trisurf(elements, ...
        vertex_Initpose(:, 1), vertex_Initpose(:, 2), vertex_Initpose(:, 3), ...
        vmStress_builtin_surface, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("3D Stress Interpolation");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Builtin Matlab von Mises stress
    subplot(1, 3, 3);
    pdeplot3D(mesh_model, 'ColorMapData', tet_VMSigma, 'FaceAlpha', 1);
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
    trisurf(elements, ...
        vertex_Initpose(:, 1), vertex_Initpose(:, 2), vertex_Initpose(:, 3), ...
        vonMisesStressNodes, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("Planar Stress Combination");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
    
    % Plot MATLAB's built-in von Mises stress interpolated to surface nodes
    subplot(1, 2, 2);
    trisurf(elements, ...
        vertex_Initpose(:, 1), vertex_Initpose(:, 2), vertex_Initpose(:, 3), ...
        vmStress_builtin_surface, 'EdgeColor', 'none');
    colorbar;
    clim([minStress, maxStress]); % Normalize color scale
    title("3D Stress Interpolation");
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); axis equal; shading interp;
end

%% Custom Von MisesStress 2D
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


%% 3D methods

function [tet_VMSigma, tet_Stress] = Compute3DvonMises(VertexInitPoses, VertexFinalPoses, MeshTetrahedrons, E, nu)
    % von Mises With our data
    n_nodes = length(VertexInitPoses);
    % Initialize displacement vector (12x1xN_tet)
    u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));

    % Compute element stiffness matrix (6x6xN_tet)
    Ce = Ce3DMatrixComputation(E, nu);
    
    % Compute strain-displacement matrices (6x12xN_tet)
    Le = Compute3DBe(MeshTetrahedrons, VertexInitPoses);
    
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
    tet_Stress = zeros(6,1,n_nodes);
    node_contributions = zeros(n_nodes, 1); % Track contributions per node
    
    % Accumulate stresses for each node
    for elem = 1:size(MeshTetrahedrons, 1)
        nodes = MeshTetrahedrons(elem, :);
        stress_elem = stresses_disp(:,:,elem);
        for i = 1:4
            tet_Stress(:,:,nodes(i)) = tet_Stress(:,:,nodes(i)) + stress_elem;
            node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
        end
    end
    
    % Average stress at each node
    for node = 1:n_nodes
        if node_contributions(node) > 0
            tet_Stress(:,:,node) = tet_Stress(:,:,node) / node_contributions(node);
        end
    end
    
    % Compute von Mises stress at each node
    tet_VMSigma = zeros(n_nodes, 1);
    for node = 1:n_nodes
        sigma_12 = tet_Stress(1,:,node) - tet_Stress(2,:,node); % sigma_x - sigma_y
        sigma_23 = tet_Stress(2,:,node) - tet_Stress(3,:,node); % sigma_y - sigma_z
        sigma_31 = tet_Stress(3,:,node) - tet_Stress(1,:,node); % sigma_z - sigma_x
        tet_VMSigma(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
            3 * (tet_Stress(4,:,node)^2 + tet_Stress(5,:,node)^2 + tet_Stress(6,:,node)^2));
    end
end

% Computation of the Elasticity Matrix (Material Stifness Matrix) from
% Hooke's law for 3D isotropic materials
function [Ce] = Ce3DMatrixComputation(E, nu)

    poissonMatrix = [1-nu nu nu 0 0 0;...
                      nu 1-nu nu 0 0 0;...
                      nu nu 1-nu 0 0 0;...
                      0 0 0 (1-2*nu)/2 0 0;...
                      0 0 0 0 (1-2*nu)/2 0;...
                      0 0 0 0 0 (1-2*nu)/2];

    Ce = E/((1+nu)*(1-2*nu)) * poissonMatrix;

end


function B = Compute3DBe(MeshTetraedrons, VertexPose)
    N_tet = length(MeshTetraedrons);
    B = zeros(6,12,N_tet);
    for e = 1:N_tet
        % Extract indices of the four vertices for the current tetrahedron
        indices = MeshTetraedrons(e, :);
        P1 = VertexPose(indices(1), :); % First point
        P2 = VertexPose(indices(2), :); % Second point
        P3 = VertexPose(indices(3), :); % Third point
        P4 = VertexPose(indices(4), :); % Fourth point

        grads = compute3DShapeFunctionGradients(P1, P2, P3, P4);
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

function grads = compute3DShapeFunctionGradients(P1, P2, P3, P4)
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
    % detJ = det(J);
    % if abs(detJ) < 1e-12
    %     error('Jacobian is singular or nearly singular. Check input points.');
    % end

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
