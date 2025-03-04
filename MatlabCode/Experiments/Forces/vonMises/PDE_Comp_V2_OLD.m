%% Data generation
gm = multicuboid(0.06,0.005,0.01);
pdegplot(gm,FaceLabels="on",FaceAlpha=0.5)
view(50,20)

% Create an femodel for transient structural analysis and include the geometry.
model = femodel(AnalysisType="structuralTransient", ...
                Geometry=gm);

% Specify Young's modulus, Poisson's ratio, and the mass density of the material.
model.MaterialProperties = ...
    materialProperties(YoungsModulus=210E9, ...
                       PoissonsRatio=0.3, ...
                       MassDensity=7800);

% Fix one end of the beam.
model.FaceBC(5) = faceBC(Constraint="fixed");

% Apply a sinusoidal displacement along the y-direction on the end opposite the fixed end of the beam.
yDisplacementFunc = ...
@(location,state) ones(size(location.y))*1E-4*sin(50*state.time);
model.FaceBC(3) = faceBC(YDisplacement=yDisplacementFunc);

% Generate a mesh.
model = generateMesh(model,Hmax=0.01,GeometricOrder="linear");
% Specify the zero initial displacement and velocity.
model.CellIC = cellIC(Displacement=[0;0;0],Velocity=[0;0;0]);

% Solve the model.
tlist = 0:0.002:0.2;
R = solve(model,tlist);

currTime = 2;

%% PDE - Von Mises
% Evaluate the von Mises stress in the beam.
vmStress = evaluateVonMisesStress(R);

% Plot the von Mises stress for the last time-step.
figure
pdeplot3D(R.Mesh,ColorMapData = vmStress(:,currTime))
title("von Mises Stress in the Beam for the Last Time-Step")

%% Custom - Von Mises
n_nodes = length(R.Mesh.Nodes);
n_times = length(tlist);

E = 210E9; % Young Modulus (Pa)
nu = 0.3; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)

VertexInitPoses = R.Mesh.Nodes';
VertexFinalPoses = zeros(n_nodes,3);

VertexFinalPoses(:,1) = VertexInitPoses(:,1) + sum(R.Displacement.x(:, 1:currTime), 2);
VertexFinalPoses(:,2) = VertexInitPoses(:,2) + sum(R.Displacement.y(:, 1:currTime), 2);
VertexFinalPoses(:,3) = VertexInitPoses(:,3) + sum(R.Displacement.z(:, 1:currTime), 2);

VertexInterPoses = zeros(n_nodes,3);

VertexInterPoses(:,1) = VertexInitPoses(:,1) + sum(R.Displacement.x(:, 1:currTime-1), 2);
VertexInterPoses(:,2) = VertexInitPoses(:,2) + sum(R.Displacement.y(:, 1:currTime-1), 2);
VertexInterPoses(:,3) = VertexInitPoses(:,3) + sum(R.Displacement.z(:, 1:currTime-1), 2);

MeshTetrahedrons = model.Geometry.Mesh.Elements';
% MeshTetrahedrons = sort(MeshTetrahedrons,2,"ascend");

volumes = abs(dot(cross(VertexInitPoses(MeshTetrahedrons(:,2),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), ...
                        VertexInitPoses(MeshTetrahedrons(:,3),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), 2), ...
                        VertexInitPoses(MeshTetrahedrons(:,4),:) - VertexInitPoses(MeshTetrahedrons(:,1),:), 2)) / 6;

% [MeshTetrahedrons,volumes] = GenerateTetrahedrons(VertexInitPoses);
[fe, fext, ~, stresses] = ForcesComputation(VertexFinalPoses, VertexInterPoses, VertexInitPoses, MeshTetrahedrons, volumes, kext, E, nu);


[VMSigma_n, VMSigma_e] = VonMisesStressComp(stresses, MeshTetrahedrons, n_nodes);
% plotVonMisesStress(MeshTetrahedrons,VertexFinalPoses,VMSigma_n);
% Plot the von Mises stress for the last time-step.
figure
pdeplot3D(R.Mesh,ColorMapData = VMSigma_n)
title("von Mises Stress Custom")

%% Error between the library and custom

relError = abs(VMSigma_n - vmStress(:, currTime)) ./ max(vmStress(:, currTime));
disp("Maximum Relative Error:");
disp(max(relError));

