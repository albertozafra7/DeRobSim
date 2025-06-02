% von Mises Stress for 3-D Structural Dynamic Problem
% Evaluate the von Mises stress in a beam under a harmonic excitation.
% Create and plot a beam geometry.
gm = multicuboid(0.06,0.005,0.01);
% pdegplot(gm,FaceLabels="on",FaceAlpha=0.5)
% view(50,20)

% Create an femodel for transient structural analysis and include the geometry.
model = femodel(AnalysisType="structuralTransient", ...
                Geometry=gm);
% Specify Young's modulus, Poisson's ratio, and the mass density of the material.
E = 210E9; % Young Modulus
nu = 0.3; % Poisson Ratio
MassDensity = 7800; % Mass Density
model.MaterialProperties = ...
    materialProperties(YoungsModulus=E, ...
                       PoissonsRatio=nu, ...
                       MassDensity=MassDensity);
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
R = evaluateTransientModel(model,tlist);
R.E = E;
R.nu = nu;
R.MassDensity = MassDensity;

% Plot the von Mises stress for the last time-step.
% figure
% pdeplot3D(R.Mesh,ColorMapData = R.vmStress(:,end))
% title("von Mises Stress in the Beam for the Last Time-Step")

tlist_eps = 0:eps:eps*1000;
R_eps = evaluateTransientModel(model,tlist_eps);
R_eps.E = E;
R_eps.nu = nu;
R_eps.MassDensity = MassDensity;

% Plot the von Mises stress for the last time-step.
% figure
% pdeplot3D(R_eps.Mesh,ColorMapData = R_eps.vmStress(:,end))
% title("von Mises Stress in the Beam for the Last Time-Step")

% Clear unnecessary variables
clear("yDisplacementFunc","tlist_eps","tlist","model","gm");

function [Results] = evaluateTransientModel(model,tlist)
    % Solve the model
    Transient_model = solve(model,tlist);
    
    % Save it in a struct
    Results.Transient_model = Transient_model;
    Results.Displacement = Transient_model.Displacement;
    Results.Velocity = Transient_model.Velocity;
    Results.Acceleration = Transient_model.Acceleration;
    Results.SolutionTimes = Transient_model.SolutionTimes;
    Results.Mesh = Transient_model.Mesh;
    
    
    % Evaluate the von Mises stress
    Results.vmStress = evaluateVonMisesStress(Transient_model);
    % Evaluate Stress
    Results.Stress = evaluateStress(Transient_model);
    % Evaluate Strain
    Results.Strain = evaluateStrain(Transient_model);
    % Evaluate Principal Stress and Strains
    Results.PrincipalStress = evaluatePrincipalStress(Transient_model);
    Results.PrincipalStrain = evaluatePrincipalStrain(Transient_model);
end