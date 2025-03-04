%% Simulation in Matlab

%% Saving things
pdfFileName = "MatlabSim";
epsFilePrefix = "";
saveEPS = false;
showFigures = false;
showLegends = true;


%% Define initial conditions for each agent
agentPose = [-9.74300003 -24.1469994;...
             9.52999973 -23.9500008;...
             9.49400043 -32.1100006;...
             -9.48999977 -32.0800018]';

agentDest = [-9.74300003 30.34;...
             9.52999973 30.34;...
             9.49400043 22.377;...
             -9.48999977 22.377]';

t_end = 1.9092e+3;   % end time
dt = 0.033;   % time step


%% Actual control

[times, positions, accels, vels, UD, UG, UH, Ugamma, Us, Ug, Uth, Uf, gamma_err, eg_err, es_err, eth_err] = simulateControl2D(agentPose, agentDest, t_end, dt);

%% Plotting
saveMatlabPlots(pdfFileName, epsFilePrefix, times, positions, agentDest, accels, vels, UD, UG, UH, Ugamma, Us, Ug, Uth, Uf, gamma_err, eg_err, es_err, eth_err, saveEPS, showFigures, showLegends)

