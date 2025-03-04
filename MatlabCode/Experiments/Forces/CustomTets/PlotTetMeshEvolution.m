% ############## Data Analysis Section ##############
%% Parameter Set
Time_id = -1; % 2;
% Plotting configuration
groundtruth = true; % Plots the comparison between methods and groundtruth
plotTrajectComp = true; % Plots the trajectory taken by the vertices/particles
plotShapeComp = false; % Plots the shape comparison between vertices and particles

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

triVertices = zeros(n_vertices,3,n_times);
triVertices(:,1,:) = x_vert(:,:)';
triVertices(:,2,:) = y_vert(:,:)';
triVertices(:,3,:) = z_vert(:,:)';

triFaces = elements;

tetraVertices = zeros(n_tet_verts,3,n_times);
tetraVertices(:,1,:) = x_tet(:,:)';
tetraVertices(:,2,:) = y_tet(:,:)';
tetraVertices(:,3,:) = z_tet(:,:)';

tetrahedra = tet_elements;

%% Plotting

% Assuming:
% - triVertices is an N_tri x 3 x T array of triangular mesh vertex positions over T time steps
% - triFaces is an F_tri x 3 array of triangular mesh face indices
% - tetraVertices is an N_tetra x 3 x T array of tetrahedral mesh vertex positions over T time steps
% - tetrahedra is an F_tetra x 4 array of tetrahedral mesh indices

% Initialize figure
figure;
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Mesh Evolution Over Time');
view(69.5797, 5.3720);


% Plot initial triangular mesh
triPlot = trisurf(triFaces, triVertices(:,1,1), triVertices(:,2,1), triVertices(:,3,1), 'FaceColor', 'cyan', 'EdgeColor', 'none');

% Plot initial tetrahedral mesh
tetraPlot = tetramesh(tetrahedra, tetraVertices(:,:,1), 'FaceAlpha', 0.1, 'EdgeColor', 'black');

% Set up lighting
light;
lighting gouraud;

% Animation loop
for t = 1:n_times
    % Update triangular mesh vertices
    set(triPlot, 'Vertices', triVertices(:,:,t));

    % Update tetrahedral mesh vertices
    tetraPlot = tetramesh(tetrahedra, tetraVertices(:,:,t), 'FaceAlpha', 0.1, 'EdgeColor', 'black');

    % Refresh plot
    drawnow;

    % Pause for a short duration to control animation speed
    pause(0.1);
end
