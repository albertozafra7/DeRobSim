%% ###### Pizza Test extracted from (Petit et all. 2017) Section 6.1 ######

% +++++ Deformation parameters ++++
E = 800; % Young Modulus (Pa)
nu = 0.3; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)

% +++++ Shape of the cylindrical object ++++
% Its dimmensions are 0.11 x 0.02 m
params.radius = 0.11/2;
params.height = 0.02;
N_vertices = 497; % Obtained from the literature

% We compute the vertices' possitions based on a standard function
VertexInitPoses = generateVertices('cylinder', params, N_vertices); % Vertex initial poses (m)
% scatter3(VertexInitPoses(:,1),VertexInitPoses(:,2),VertexInitPoses(:,3));
% We compute the tetrahedrons of the mesh based on its vertices
% [MeshTetrahedrons,volumes] = GenerateTetrahedrons(VertexInitPoses);
% tetramesh(MeshTetrahedrons, VertexInitPoses, 'FaceAlpha', 0.3);


%  +++++ Forces Applied ++++
% Stretch Point
Point1 = VertexInitPoses(235,:);
% Bending Points
Point2 = VertexInitPoses(222,:);
Point3 = VertexInitPoses(194,:);
% Fixed Points
Point4 = VertexInitPoses(209,:);
Point5 = VertexInitPoses(208,:);
Point6 = VertexInitPoses(207,:);

grabbing_point = Point1;
bending_points = [Point2;Point3];
fixed_points = [Point4;Point5;Point6];

% Parameters for deformation
stretch_amount = -0.05;  % Stretching distance in -X direction
bend_angle = pi / 6;              % Total bend angle (in radians, e.g., 30 degrees)
bending_axis = 'z';               % Bending around Z-axis

% Extract original vertices
original_vertices = VertexInitPoses;  % Keep a copy for comparison

% Deform the cylinder according to a stretch in the X axis
stretched_points = stretchFromPoint(original_vertices, grabbing_point, fixed_points, nu, stretch_amount);

% Deform the cylinder according to a bend in the Z axis
bended_points = bendFrom2Points(stretched_points,bending_points, bend_angle, bending_axis);

% Plot the evolution of the deformations
plotVerticesComp(original_vertices,bended_points,["Original Cylinder";"Deformed Cylinder (Stretching & Bending)"]);

% Plot the evolution of the deformations with tetramesh
plotMeshComp(original_vertices,bended_points, MeshTetrahedrons, ["Original Cylinder";"Deformed Cylinder (Stretching & Bending)"]);

%  +++++ Forces Computation ++++
[fe,fext] = ForcesComputation(bended_points, original_vertices, original_vertices, MeshTetrahedrons, volumes, 1, 1, 0.25);

% Plot of the forces
plotForces(bended_points,MeshTetrahedrons,fe,fext);
