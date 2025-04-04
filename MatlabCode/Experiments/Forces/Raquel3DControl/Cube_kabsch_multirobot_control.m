%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%% CUBE MESH %%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc; 

%% General Script configuration
% Trajectory config
waypoints = false;
rotation_z = false;
having_obstacles = false;
moveNdim = 1; % 1 -> Movement in X; 2 -> Movement in X and Y; 3 -> Movement in 3D

% If we want to use the Control Barrier Functions
forceCBF = false;
realCBF = true;

% If we want to plot the control results
plotTetramesh = false;
plotCResults = false;
plotCResultsCBF = true;
plotCBFDifferences = true;
plotControlOutputs = false;
plotVelComp = true;
plotStresses = true;

%% We add the paths of the folder and subfolders we will need.

load('.\Experiments\Forces\MatlabInteractiveTests\JacobianUnitaryCube.mat');
clear agent_actions mesh_model n_experiments particle_displacements vonMisesValues;

Aux_path = genpath('Multirobot_control_3D_aux_files');
if(isempty(Aux_path))
    disp('WARNING: Cannot find utilities directory.  This script should be run from the base directory.')
else
    addpath(Aux_path)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Choice of general conditions of the simulation

ob_type = 1;            % select type of object: 1 for sheet, 2 for linear (rod, cable)
with_object = 0;        % 0: simulation without object; 1: with object
with_obstacles = 0;     % 0: without obstacles; 1: with obstacles
with_waypoints = 0;     % 0: without waypoints; 1: with waypoints

show_figures = 1;       % 0: does not show figures during execution; 1: shows them
show_rest_shape = 0;    % 1: plots the object's rest shape and node indexes (we can use this plot to define the nodes where the robots are placed)

save_variables = 0;     % 0: does not save variables; 1: save simulation results; 2: save Robotarium results
    variables_name = 'var_mattress_K_with_G';
save_plots = 0;         % 0: does not save plots; 1: save plots
save_frames = 0;        % 0: does not save frames for the video; 1: save frames
make_video = 0;         % 0: does not make a video; 1: makes it
    v_name = 'vid_mattress_K_with_G';    % video file name
create_folder = 0;      % 0: does not create a folder; 1: creates a folder
    f_name = 'mattress_K_with_G';     % folder name for storing the results 
    
if create_folder == 1
    folder_result = strcat('./Object_',string(ob_type),'_',f_name,'_sim','/');  % it's a path
    if exist(char(folder_result),'dir') == 0                            % 'dir' - Just folders
       mkdir(char(folder_result));                                      % create the folder
    end
end


%% Choice of general parameters of the simulation

N = 8; % number of robots
ndims = 3; % Number of dimensions

% We define some matrices we will use in our formulation
K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix (3*Na x 3*Na)
S = [0 -1; 1 0];
T = kron(eye(N),S);

% Time parameters - The simulated time in seconds will be nit*dt
dt = 0.01;      % time step of the simulation
niters = 700;%1600;  % number of iterations of the control loop

% Physical parameters
E = 3000; % Young Modulus (Pa)
nu = 0.01; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)
yield_stress = 210e2; % Yield Stress (Pa)
% SF = 0.0005;
SF = 2.5;%1.5;%0.8; % Safety Factor (Unitless)


%% We define the object's nodes arranged in a rectangular grid

nx = 5; % number of nodes in x direction of the grid
ny = 5;% number of nodes in y direction of the grid
nz = 5; % number of nodes in z direction of the grid

sx = 1; % length of the object in x direction (in the graphics, it will be the 'y' direction)
sy = 1; % length of the object in y direction (only relevant if ob_type = 1) (in the graphics, it will be the x direction)
sz = 1; % length of the object in z direction

% We need to reduce the defined object for the Robotarium experiment because its
% working area is smaller.
scale_ob = 1;

omesh = create_mesh3D(nx, ny, nz, sx * scale_ob, sy * scale_ob, sz * scale_ob); % mesh of the object: its shape and the links between nodes
% We put the object over the Z axis (which means that the lowest part of
% the body will be at 0 or over, regarding the Z axis, so it does not go
% under the floorte
omesh.shape = omesh.shape + [0, 0, -1*(min(omesh.shape(:,3))<0)*min(omesh.shape(:,3))];

if plotTetramesh
    figure;
    tetramesh(omesh.elements, omesh.shape);
end

%% Get the tetrahedrons
% X = omesh.shape(:,1);
% Y = omesh.shape(:,2);
% Z = omesh.shape(:,3);
% 
% shp = alphaShape(X(:), Y(:), Z(:));
% surfTriangles = boundaryFacets(shp);
% gm = fegeometry([X(:), Y(:), Z(:)], surfTriangles);
% model = femodel(AnalysisType="structuralStatic", Geometry=gm);
% model.MaterialProperties = materialProperties(YoungsModulus=210e10, PoissonsRatio=0.3, MassDensity=2.7e-6);
% model = generateMesh(model, GeometricOrder="linear");
% % initialTetPositions = model.Geometry.Mesh.Nodes'; % Transpose for [x, y, z] format
% 
% %% Save the new mesh data in omesh
% % omesh contains the node positions (
% 
% % Node positions
% omesh.shape = model.Geometry.Mesh.Nodes'; % (Npx3)
% omesh.shape = omesh.shape + [0, 0, -1*(min(omesh.shape(:,3))<0)*min(omesh.shape(:,3))];
% % Mesh Model
% omesh.meshModel = model;
% % Mesh Tetrahedra
% omesh.elements = model.Geometry.Mesh.Elements'; % (Ntetx4)

%% Definition of reference, initial and target configurations

% We define the parameters of the initial and desired configurations
% (centroids, scales and orientations) for each experiment in order to
% make it easier to select them.

% Initial node positions
initialTetPositions = omesh.shape; % (Npx3)
Pob0 = omesh.shape'; % (3xNp)

% Agent's Initial Positions (Each one at a vertex of the object)
p_init = [ 0.5,  0.5,  1.0;... % Agent 1
           0.5, -0.5,  1.0;... % Agent 2
          -0.5, -0.5,  1.0;... % Agent 3
          -0.5,  0.5,  1.0;... % Agent 4
           0.5,  0.5,  0.0;... % Agent 5
           0.5, -0.5,  0.0;... % Agent 6
          -0.5, -0.5,  0.0;... % Agent 7
          -0.5,  0.5,  0.0];   % Agent 8

% Object's Initial Centroid Position (Mean of the Agent's positions for this problem)
g0 = mean(p_init)'; % (3x1)

g0_rep = repmat(g0',N,1);
p_agentsFrom_g0 = p_init - g0_rep; % (Nax3)


% Initial Scale Factor
s0 = 1; % No scaling

% Initial Rotation Angles
th0_x = 0*pi/180; % All set to 0 rad
th0_y = 0*pi/180;
th0_z = 0*pi/180;

switch moveNdim
    case 1
        % Desired (target) Centroid Position
        gd = g0 + [-2.7, 0, 0]'; % We want to have a movement in the X axis (but rotating the object)
        gd_final = gd; % Used later in the control logic (waypoint handling)
    case 2
        % Desired (target) Centroid Position
        gd = g0 + [-2.7, 3.0, 0]'; % We want to have a movement in the X and the Y axis (but rotating the object)
        gd_final = gd; % Used later in the control logic (waypoint handling)
    otherwise
        % Desired (target) Centroid Position
        gd = g0 + [-2.7, 3.0, 1.0]'; % We want to have a movement in the 3 axis (but rotating the object)
        gd_final = gd; % Used later in the control logic (waypoint handling)
end

% Desired Scale Factor
sd0 = 1; % No Change

% Desired Rotation Angles
thd0_x = 0*pi/180;    % 0 deg
thd0_y = 0*pi/180;    % 0 deg
if rotation_z
    thd0_z = -180*pi/180; % Rotation of 180 deg in the Z axis
else
    thd0_z = 0*pi/180;
end

%% Definition of the initial configuration
% We define the initial configuration
p_init = reshape(p_init', [3*N, 1]); % (3*Na x 1)
p00 = K * p_init; % (3*Na x 1)
R0 = eul2rotm([th0_x th0_y th0_z], 'XYZ');
R0 = kron(eye(N), R0);
p0 = (s0 * R0 * p00) + reshape(g0 * ones(1,N), [3*N,1]); % (3*Na x 1)

% We define the reference configuration, c
c = K * p00; % We want to preserve its shape
c = K * c; % reference configuration with centroid at (0,0) % (3*Na x 1)

% We define the desired configuration
Rd0 = eul2rotm([thd0_x thd0_y thd0_z], 'XYZ');
Rd0 = kron(eye(N), Rd0);
pd = (sd0 * Rd0 * c);

%% Desired Set-Up of the agents
% Target formation scale and rotation. Since we have already rotated and scaled
% pd when we defined it above, then we can select sd = 1 and thd = 0.
sd = 1;
thd = 0;            
Rd = eul2rotm([thd thd thd], 'XYZ');
Rd_final = Rd;
Rd_T = kron(eye(N), Rd);
H_kd = sd * Rd; % we need this matrix to control the rotation
PT = (sd * Rd_T * pd) + reshape(gd * ones(1,N), [3*N,1]);     % final configuration with centroid in gd (3*Na x 1)

% Control terms are calculated with c. As the code is defined here, we can
% replace c with pd in the control terms code or do the following:
c = K * PT; % (3*Na x 1)


%% Waypoints definition

if waypoints
    num_waypoint = 1;
    
    % First waypoint
    g1 = g0 + [0, -1.3, 0.6]'; % Waypoint centroid (3x1)
    s1 = 1;                                         
    th1_x = 0*pi/180;
    th1_y = -30*pi/180;
    th1_z = 0*pi/180;
    R1 = eul2rotm([th1_x th1_y th1_z], 'XYZ');  
    R1 = kron(eye(N), R1);                         
    c1 = (s1 * R1 * p00); % Waypoint Agents' positions (3*Na x 1)
    
    % Second waypoint
    g2 = g0 + [-1.35, -2.55, 1.1]'; %  Waypoint centroid (3x1)
    s2 = 1;                                         
    th2_x = -45*pi/180;
    th2_y = 0*pi/180;
    th2_z = -90*pi/180;
    R2 = eul2rotm([th2_x th2_y th2_z], 'XYZ');  
    R2 = kron(eye(N), R2);                         
    c2 = (s2 * R2 * p00); % Waypoint Agents' positions (3*Na x 1)
    
    % Third waypoint
    g3 = g0 + [-2.7, -1.3, 0.6]'; %  Waypoint centroid (3x1)
    s3 = 1;                                         
    th3_x = 0*pi/180;
    th3_y = 30*pi/180;
    th3_z = -180*pi/180;
    R3 = eul2rotm([th3_x th3_y th3_z], 'XYZ');  
    R3 = kron(eye(N), R3);                         
    c3 = (s3 * R3 * p00); % Waypoint Agents' positions (3*Na x 1)
    
    waypoints_g = [g1 g2 g3 gd]; % Waypoint centroids (3xNwaypoints)
    waypoints_c = [c1 c2 c3 c]; % Waypoints Agents' positions (3*Na x Nwaypoints)
    
    gd = waypoints_g(:, num_waypoint); % Current desired waypoint centroid (3x1)
    c = waypoints_c(:, num_waypoint); % Current desired Agents' positions (3*Na x 1)
end

%% Obtacles definition
if having_obstacles
    % Internal wall
    r1 = 0.2;   
    initial_point1 = g0 + [-1.35+r1; 1.5; -0.15];
    second_point1 = g0 + [-1.35+r1; -1.2; -0.15];
    third_point1 = g0 + [-1.35-r1; -1.2; -0.15];
    final_point1 = g0 + [-1.35-r1; 1.5; -0.15];
    points = initial_point1;
    obstacles = [];
    
    while points(2) > second_point1(2)
        obstacles = [obstacles, points];
        points(2) = points(2) - 0.1;
    end
    
    obstacles = [obstacles, points, second_point1];
    
    for th1 = pi:pi/5:2*pi
        points(2) = r1 * sin(th1) + g0(2) - 1.2;
        points(1) = -r1 * cos(th1) + g0(1) - 1.35;
        obstacles = [obstacles, points];
    end
    
    obstacles1 = [obstacles, third_point1, final_point1];
    
    % External wall
    r2 = 2.5;   
    initial_point2 = g0 + [-1.35+r2; -1.2; -0.15];
    final_point2 = g0 + [-1.35-r2; -1.2; -0.15];
    points = initial_point2;
    obstacles = g0 + [-1.35+r2; 1.5; -0.15];
    
    for th2 = 2*pi:-pi/10:pi
        obstacles = [obstacles, points];
        points(2) = r2 * sin(th2) + g0(2) - 1.2;
        points(1) = r2 * cos(th2) + g0(1) - 1.35;               
    end
    
    obstacles2 = [obstacles, final_point2, g0 + [-1.35-r2; 1.5; -0.15]];
end          

%% Initial conditions of the simulation and required simulation parameters

% Initial positions of the robots
p = p0;         % p matrix 3Nx1
p_cbf = p0;     % p_cbf matrix 3Nx1

% Velocity limit value
u_sat = 1.5;    % m/s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gain values for the control
k_H = 50;        % for moving toward shape-preserving transformation
k_G = 5;         % for moving toward a configuration consistent with our deformation modes
k_c = 1;         % for centroid translation
k_s = 50;        % for scaling
k_Hd = 1.5;      % for scaling and orientation
kCBF = 1;        % for controllig the stress avoidance influence
cbf_alpha = 1;   % for controlling the risks taken by the barrier function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Each of the following will be a growing array that will store the values of variables at each iteration
ps = [];                % positions
gammas_H = [];          % cost relative to shape-preserving transformation
gammas_G = [];          % cost relative to configuration consistent with our deformation modes
hs = [];                % parameters of the optimal shape-preserving transformation
egs = [];               % centroid errors
ess = [];               % scale errors
eths = [];              % rotation eror
eth_xs = [];            % x angle errors
eth_ys = [];            % y angle errors
eth_zs = [];            % z angle errors
dx_unis = [];           % unicycle velocities
vs = [];                % linear velocities
nvs = [];               % linear velocities norm
vm_stresses = [];       % von Mises stresses
tensor_stresses = [];   % stress tensors
u_Hs = [];              % shape-preserving control (UH)
u_Gs = [];              % Deformation Control (UG)
U_ss = [];              % Scale Control (US)
u_cs = [];              % Position Control (Uc)
U_Hds = [];             % Scale and Orientation Control (UHd)
u_cbfs = [];            % Control Barrier Functions
us = [];                % Final Control Outputs (U_f)

% CBF
ps_cbf = [];            % positions with the barrier function
nvs_cbf = [];           % linear velocities norm CBF
vs_cbf = [];            % linear velocities
As = [];                % Left side of the optimization condition
bs = [];                % Right side of the optimization condition
gammas_H_cbf = [];      % cost relative to shape-preserving transformation
gammas_G_cbf = [];      % cost relative to configuration consistent with our deformation modes
egs_cbf = [];           % centroid errors
ess_cbf = [];           % scale errors
eths_cbf = [];          % rotation eror
eth_xs_cbf = [];        % x angle errors
eth_ys_cbf = [];        % y angle errors
eth_zs_cbf = [];        % z angle errors

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% For creating the video:
ind_im = 1;     % index of images (used for creation of video)
fps = 20;       % frames per second for the video
% Create figure from which we will create the video frames
if show_figures == 0
    video = figure(20);
    set(video, 'visible', 'off')
end

%% Plotting

plotaxfont='Helvetica';
color_robots = ["#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7", "#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7"]; % 14 robots
color_graphs = ["#0073bd", "#77a5c3", "#d9541a", "#edb120", "#7e2f8e"];

% Limits
if having_obstacles
    x_limit = [-3.1 1.9];
    y_limit = [-3.1 1.45];
    z_limit = [0 3];
else
    x_limit = [-4 2];
    y_limit = [-1 4];
    z_limit = [0 3];
end

zoom_factor = 1.25;  % greater than 1 means zoom out
x_limit = x_limit * zoom_factor;
y_limit = y_limit* zoom_factor;
z_limit = z_limit * zoom_factor;

% Create animation (Considering only 2 layers of Agents)
pairs = [];
for i = 1:N/2-1
    pairs = [pairs; [i i+1]; [N/2+i N/2+i+1]];
end
pairs = [pairs; [1 N/2]; [N/2+1 N]]; % close the shape in the sheet object

% Connecting layers vertically
for i = 1:N/2
    pairs = [pairs; [i i+N/2]];
end

% Connecting oposite vertices whose line cross at the centroid of the object
pairs = [pairs; [1 7]; [2 8]; [3 5]; [4 6]];
%%%

if having_obstacles
    for i = 1:size(pairs,1)
        object_lines(i) = plot3([p(3*pairs(i,1)-2), p(3*pairs(i,2)-2)], [p(3*pairs(i,1)-1), p(3*pairs(i,2)-1)], [p(3*pairs(i,1)), p(3*pairs(i,2))], 'k-', 'linewidth', 0.5);
    end
    
    % Obtacles plotting
    for i = 0:50
        plot3(obstacles1(1,:), obstacles1(2,:), obstacles1(3,:) + i*0.02, 'k-', 'LineWidth', 2);
    end
    for i = 0:10
        plot3(obstacles2(1,:), obstacles2(2,:), obstacles2(3,:) + i*0.02, 'k-', 'LineWidth', 2);
    end
end

if waypoints
    % Initial, target and current positions
    for i = 1:N
        plot3(p0(3*i-2,1), p0(3*i-1,1), p0(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 15);
        plot3(PT(3*i-2,1), PT(3*i-1,1), PT(3*i,1), 'o', 'color', color_robots(i), 'MarkerSize', 7, 'LineWidth', 1.5);
        robot_tr(i) = plot3(p(3*i-2,1), p(3*i-1,1), p(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 30);
    end
    plot3(g0(1,:), g0(2,:), g0(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    % plot3(gd(1,:), gd(2,:), gd(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    plot3(waypoints_g(1,:), waypoints_g(2,:), waypoints_g(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
end


%% Object Simulation
handled_nodes = [];
for i = 0:N-1
    differences = omesh.shape - p_init(3*i+1:3*i+3)';       % Subtract target from each column
    dists = sqrt(sum(differences.^2, 2)); % Euclidean distances for each column
    [~, idx] = min(dists);          % minDist is the minimum distance, idx is the column index
    handled_nodes = [handled_nodes, idx];
end

n_iters_arap = 3; % choose a value of 2 or 3: the higher the value the stiffer the simulated object

arap_params = create_params_for_arap(omesh, handled_nodes, [], n_iters_arap);  % create model parameters

% Simulate the initial position of the object's nodes
Pob = simulate_object_arap(arap_params, omesh.shape', [reshape(p, [3,N])]); % (3xNp)
Pob_cbf = Pob;

%% Object plotting initialization

if with_object == 1

    for i = 1:size(arap_params.edg, 1)   % we plot the edges of the sheet's mesh between nodes
        object_mesh(i) = plot3([Pob(1,arap_params.edg(i,1)) Pob(1,arap_params.edg(i,2))], ...
                     [Pob(2,arap_params.edg(i,1)) Pob(2,arap_params.edg(i,2))], ...
                     [Pob(3,arap_params.edg(i,1)) Pob(3,arap_params.edg(i,2))], ...
                     '-', 'color', [0.5 0.5 0.8], 'linewidth', 1);
    end
                       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control loop
for it_loop = 1:niters

    % Store current positions
    ps = [ps, p]; % (3*Na x N_iters)
    ps_cbf = [ps_cbf, p_cbf]; % (3*Na x N_iters)
    
    % Current formation centroid
    g = (1/N) * [sum(p(1:3:end)); sum(p(2:3:end)); sum(p(3:3:end))]; % (3x1)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % For rotating the view
    if (it_loop > 25) && (it_loop < 550) && having_obstacles
        view(view_1-(it_loop-25)*0.5, view_2)
    end
    
    if having_obstacles
        % Plot the robots in their current position in each iteration
        for i = 1:N
            robot_tr(i).XData = p(3*i-2,1);
            robot_tr(i).YData = p(3*i-1,1);
            robot_tr(i).ZData = p(3*i,1);
        end
        
        for i = 1:size(object_lines,2)
            object_lines(i).XData = [p(3*pairs(i,1)-2), p(3*pairs(i,2)-2)];
            object_lines(i).YData = [p(3*pairs(i,1)-1), p(3*pairs(i,2)-1)];
            object_lines(i).ZData = [p(3*pairs(i,1)), p(3*pairs(i,2))];
        end
    end

    % Simulate how the object is deformed
    Pob = simulate_object_arap(arap_params, Pob, [reshape(p, [3,N])]);
    Pob_cbf = simulate_object_arap(arap_params, Pob_cbf, [reshape(p_cbf, [3,N])]);

    if with_object == 1

        for i = 1:size(arap_params.edg, 1)
            object_mesh(i).XData = [Pob(1,arap_params.edg(i,1)) Pob(1,arap_params.edg(i,2))];
            object_mesh(i).YData = [Pob(2,arap_params.edg(i,1)) Pob(2,arap_params.edg(i,2))];
            object_mesh(i).ZData = [Pob(3,arap_params.edg(i,1)) Pob(3,arap_params.edg(i,2))];
        end
                
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if waypoints
        if num_waypoint < size(waypoints_g,2)
            if norm(gd - g) < 0.2
                num_waypoint = num_waypoint + 1;
                gd = waypoints_g(:, num_waypoint);
                c = waypoints_c(:, num_waypoint);
            end
        end
    end

    agent_destinations = p_agentsFrom_g0 + repmat(gd',N,1);
    agent_destinations = agent_destinations'; % (3xNa)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Control Algorithm

    if realCBF
        [~, ~, ~, ~, ~, ~, ~, u_cbf, agent_cbf_positions, ~, ~, ~, ~, ~, ~, ~, vm_stress, stress_tensor, A, b] = ForceControl3D_Debug(Pob0', Pob_cbf', reshape(p_cbf, [ndims,N]), agent_destinations, omesh.elements, J, E, nu, yield_stress, SF, kCBF, cbf_alpha, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
        vm_stresses = [vm_stresses, vm_stress];
        tensor_stresses = cat(3, tensor_stresses, stress_tensor);
    end
    [u, U_f, U_H, u_G, U_s, u_c, U_Hd, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual] = TransportationControl3D_Debug(reshape(p, [ndims,N]), agent_destinations, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% CBF

    % Compute the MaxStress
    MaxStress = yield_stress/SF;

    H = 2 * eye(3*N);
    f = -2 * u(:);

    if ~realCBF
        if it_loop > niters/2 && forceCBF
            % A*u >= b --> La cbf actúa
            A = ones(N,3*N);
            b = - cbf_alpha * ones(N,1) * MaxStress;
        else
            % A*u <= b --> La cbf no actúa
            A = - eye(N,3*N);
            b = cbf_alpha * ones(N,1) * MaxStress;
        end
        % Set optimization options
        options = optimset('Display', 'off');
    
        % Solve QP for velocity
        u_cbf = quadprog(H, f, A, b, [], [], [], [], [], options);
        % [u_cbf, fval, exitflag] = quadprog(H, f, A, b);
        % u_cbf = kCBF * u_cbf;

    end
    As = [As, A*u_cbf];
    bs = [bs, b];


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Errors

    % Store current control errors
    gammas_H = [gammas_H; gamma_H];
    gammas_G = [gammas_G; gamma_G];
    egs = [egs; eg];                                % Centroid error
    ess = [ess; es];                                % Scale error

    eths = [eths; eth];                             % Rotation error
    eth_xs = [eth_xs; eth_individual(1)];           % Roll Error
    eth_ys = [eth_ys; eth_individual(2)];           % Pitch Error
    eth_zs = [eth_zs; eth_individual(3)];           % Yaw Error

    u_Hs = cat(3, u_Hs, U_H);                       % shape-preserving control (UH)
    u_Gs = cat(3, u_Gs, reshape(u_G,[3,N]));        % Deformation Control (UG)
    U_ss = cat(3, U_ss, U_s);                       % Scale Control (US)
    u_cs = cat(3, u_cs, reshape(u_c,[3,N]));        % Position Control (Uc)
    U_Hds = cat(3, U_Hds, U_Hd);                    % Scale and Orientation Control (UHd)
    u_cbfs = cat(3, u_cbfs, reshape(u_cbf,[3,N]));  % Control Barrier Functions
    us = cat(3, us, reshape(u,[3,N]));              % Standard Final Output


    %%%%%%%%%%%%%%%%%%%%%%% 
    
    u = u(:);

    p = p + u*dt; % apply control input
    p_cbf = p_cbf + u_cbf*dt;
    
    % Velocity norm
    for i = 1:N 
        nv(i,1) = norm(u(3*i-2:3*i,1)); % we store the norms of velocity
        nv_cbf(i, 1) = norm(u_cbf(3*i-2:3*i,1));
    end
    
    % Store current velocity data
    vs = [vs u];
    nvs = [nvs nv];

    vs_cbf = [vs_cbf u_cbf];
    nvs_cbf = [nvs_cbf nv_cbf];

    %% CBF Errors
    [g_Hcbf, g_Gcbf, eg_cbf, es_cbf, eth_cbf, eth_ind_cbf] = compute_errors3D(reshape(p_cbf,[3,N]),reshape(PT,[3,N]),sd, thd);

    % Store current control errors
    gammas_H_cbf = [gammas_H_cbf; g_Hcbf];
    gammas_G_cbf = [gammas_G_cbf; g_Gcbf];
    egs_cbf = [egs_cbf; eg_cbf];                         % Centroid error
    ess_cbf = [ess_cbf; es_cbf];                         % Scale error

    eths_cbf = [eths_cbf; eth_cbf];                      % Rotation error
    eth_xs_cbf = [eth_xs_cbf; eth_ind_cbf(1)];           % Roll Error
    eth_ys_cbf = [eth_ys_cbf; eth_ind_cbf(2)];           % Pitch Error
    eth_zs_cbf = [eth_zs_cbf; eth_ind_cbf(3)];           % Yaw Error
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % We save each frame for creating the video
    if save_frames == 1  % we simulate the behavior of the real object
        
        % We create the frame for the video, including object and robots
        if it_loop==1 || rem(it_loop, floor(1/(fps*dt))) < 1e-8    % we do this because checking "rem == 0" can be unreliable

            % We can save the figure and print it into image files (eps, png)
            hgsave(strcat(folder_result,'/','robots_object'));
            print(strcat(folder_result, '/', 'robots_object_', int2str(ind_im)), '-dpng', '-r300');
            ind_im = ind_im + 1;    % increase the index for the next frame

        end
    end
    
    % pause(0.005) % we give some time for the user to visualize the created plot
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% We save all variables of the workspace
if save_variables == 1
    save(fullfile(strcat(folder_result, '/', variables_name, '_3D')));
end

%% Create plotting and video
% Viewing angles for the 3D plot
view_1 = 0; % 110 deg
view_2 = 90;  % 29 deg
view_1 = 14.1759;
view_2 = 18.4354;
save_plots = false;

%% Plot standard data
SimData2Struct_3DControl;
if plotCResults
    plotControlResults(data_standard);
end

%% Plot CBF data
data_cbf = data_standard;
% Update data values
data_cbf.plotTitleSufix = 'CBF';
data_cbf.ps = ps_cbf;
data_cbf.nvs = nvs_cbf;
data_cbf.vs = vs_cbf;
data_cbf.gammas_H = gammas_H_cbf;
data_cbf.gammas_G = gammas_G_cbf;
data_cbf.egs = egs_cbf;
data_cbf.ess = ess_cbf;
data_cbf.eths = eths_cbf;
data_cbf.eth_xs = eth_xs_cbf;
data_cbf.eth_ys = eth_ys_cbf;
data_cbf.eth_zs = eth_zs_cbf;

if plotCResultsCBF
    plotControlResults(data_cbf);
end

%% Plot control outputs
if plotControlOutputs
    plot_control_outputs;
end

%% Plot the CBF diferences
if plotCBFDifferences
    figure;
    
    for a = 1:N
        plot(As(a,:),'Color', color_robots(a),'DisplayName',strcat('A x u_',num2str(a)), 'LineStyle','-');
        hold on;
        plot(bs(a,:), 'Color', color_robots(a), 'DisplayName', strcat('b agent_',num2str(a)), 'LineStyle','--');
    end
    hold off;
    title("A x u_c_b_f <= b");
    xlabel("Iters");
    ylabel("unitless");
    legend show;
end

%% Plot velocities

if plotVelComp

    figure
    plot(nvs(1,:)');
    hold on;
    plot(nvs_cbf(1,:)');
    hold off;
    title("Velocity Norm Comparison");
    xlabel("Iters");
    ylabel("m/s");
    % legend(["Agend 1 - std", "Agend 2 - std", "Agend 3 - std", "Agend 4 - std", "Agend 1 - cbf", "Agend 2 - cbf", "Agend 3 - cbf", "Agend 4 - cbf"]);
    legend(["Agend 1 - std", "Agend 1 - cbf"]);
end

%% Plot stresses
if plotStresses
    figure;
    
    plot(max(vm_stresses),'Color','b','DisplayName', "currStress");
    hold on;
    plot(repelem(yield_stress/SF,size(vm_stresses,2)),'Color', 'r', 'DisplayName', "maxStress");
    hold off;
    title("Von Mises Stress Comp");
    legend("show");
    xlabel("Iters");
    ylabel("Pa");
end