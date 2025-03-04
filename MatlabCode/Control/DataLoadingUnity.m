%% This is the test code for the Transportation control function
agentPose = [-9.74300003 1.19000006 -24.1469994;...
             9.52999973 1.19000006 -23.9500008;...
             9.49400043 1.19000006 -32.1100006;...
             -9.48999977 1.19000006 -32.0800018];

agentDest = [-9.74300003 1.19000006 -7;...
             9.52999973 1.19000006 -6.8030014;...
             9.49400043 1.19000006 -14.9630013;...
             -9.48999977 1.19000006 -14.9330025];

agentPrevPose = agentPose;

% We try to execute the function with default values
disp(TransportationControl([agentPose(:,1) agentPose(:,3)]',[agentDest(:,1) agentDest(:,3)]',[agentPrevPose(:,1) agentPrevPose(:,3)]'))

% Resultados de aceleración proporcionados por Unity
% X -->  70.0179  -26.3399   14.2382  -47.7220
% Y -->    0         0         0         0

% Resultados de aceleración proporcionados por Matlab
% X --> 1.0e-13 *   -0.1066    0.1066    0.1066   -0.1066
% Y -->  0         0         0         0


