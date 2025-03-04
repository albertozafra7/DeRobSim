% ############## Data Analysis Section ##############
%% Parameter Set
Time_id = -1; % 2;

%% Data Conversion
%hdf52mat('D:\alber\Documents\Cosas de la Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\Experiments\Forces\Unity\BeamSim.h5');
load('D:\alber\Documents\Cosas de la Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\Experiments\Forces\Unity\BeamSim.mat');

%% Data Preparation
defBeam = Unity("DefBeam");
particleInfo = defBeam("particles_info");
vertexInfo = defBeam("vertex_info");

n_times = length(times);
n_vertices = defBeam("n_vertices");
n_vertices = n_vertices(1);
n_particles = defBeam("n_particles");
n_particles = n_particles(1);
% Preallocate arrays for each field
x_part = zeros(n_times, n_particles, 'single');
y_part = zeros(n_times, n_particles, 'single');
z_part = zeros(n_times, n_particles, 'single');

x_vert = zeros(n_times, n_vertices, 'single');
y_vert = zeros(n_times, n_vertices, 'single');
z_vert = zeros(n_times, n_vertices, 'single');

% Populate the arrays
for i = 1:n_times
    x_part(i, :) = particleInfo(i).x';
    y_part(i, :) = particleInfo(i).y';
    z_part(i, :) = particleInfo(i).z';

    x_vert(i, :) = vertexInfo(i).x';
    y_vert(i, :) = vertexInfo(i).y';
    z_vert(i, :) = vertexInfo(i).z';
end

if Time_id > n_times || Time_id < 1
    Time_id = n_times;
end

%% data Plotting
% Both Traject comp
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

% +++++ Deformation parameters ++++
E = 800; % Young Modulus (Pa)
nu = 0.3; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)
%%
% We compute the tetrahedrons of the mesh based on its vertices
[Part_MeshTetrahedrons,Part_volumes] = GenerateTetrahedrons(particle_Initpose);
[Vert_MeshTetrahedrons,Vert_volumes] = GenerateTetrahedrons(vertex_Initpose);
% tetramesh(MeshTetrahedrons, VertexInitPoses, 'FaceAlpha', 0.3);

% Plot the evolution of the deformations
% plotVerticesComp(VertexInitPoses,VertexFinalPoses,["Original obj";"Deformed obj (Stretching)"]);

% Plot the evolution of the deformations with tetramesh
% plotMeshComp(VertexInitPoses,VertexFinalPoses, MeshTetrahedrons, ["Original obj";"Deformed obj (Stretching)"]);

%  +++++ Forces Computation ++++
[part_fe, part_fext, ~, part_stresses] = ForcesComputation(particle_Finalpose, particle_Initpose, particle_Initpose, Part_MeshTetrahedrons, Part_volumes, kext, E, nu);
[vert_fe, vert_fext, ~, vert_stresses] = ForcesComputation(vertex_Finalpose, vertex_Initpose, vertex_Initpose, Vert_MeshTetrahedrons, Vert_volumes, kext, E, nu);

% Plot of the forces
% plotForces(VertexFinalPoses,MeshTetrahedrons,fe,fext);

[part_VMSigma_n, part_VMsigma_e] = VonMisesStressComp(part_stresses, Part_MeshTetrahedrons, n_particles);
[vert_VMSigma_n, vert_VMsigma_e] = VonMisesStressComp(vert_stresses, Vert_MeshTetrahedrons, n_vertices);

%%

plotVonMisesStress(Part_MeshTetrahedrons,particle_Finalpose,part_VMSigma_n, "Particles");
plotVonMisesStress(Vert_MeshTetrahedrons,vertex_Finalpose,vert_VMSigma_n, "Vertices");


