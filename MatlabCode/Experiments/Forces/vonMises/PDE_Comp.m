%% Data generation
gm = multicuboid(0.06,0.005,0.01);
figure
pdegplot(gm,FaceLabels="on",FaceAlpha=0.5)
view(50,20)

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

% currTime = length(tlist);

%% PDE - Von Mises
% Evaluate the von Mises stress in the beam.
% vmStress = evaluateVonMisesStress(R);

% Plot the von Mises stress for the last time-step.
figure
pdeviz(R.Mesh,R.VonMisesStress)
title("von Mises Stress in the Beam for the Last Time-Step")

%% Custom - Von Mises
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

% [MeshTetrahedrons,volumes] = GenerateTetrahedrons(VertexInitPoses);
[fe, fext, ~, stresses] = ForcesComputation(VertexFinalPoses, VertexInterPoses, VertexInitPoses, MeshTetrahedrons, volumes, kext, E, nu);


[VMSigma_n, VMSigma_e] = VonMisesStressComp(stresses, MeshTetrahedrons, n_nodes);
% plotVonMisesStress(MeshTetrahedrons,VertexFinalPoses,VMSigma_n);
% Plot the von Mises stress for the last time-step.
figure
pdeviz(R.Mesh,VMSigma_n)
title("von Mises Stress Custom")

%% Error between the library and custom

relError = abs(VMSigma_n - R.VonMisesStress) ./ max(R.VonMisesStress);
disp("Maximum Relative Error:");
disp(max(relError));

