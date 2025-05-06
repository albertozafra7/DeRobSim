%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%% CUBE MESH %%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc; 

%% Add path of the aux files
Aux_path = genpath('Multirobot_control_3D_aux_files');
if(~isempty(Aux_path))
    addpath(Aux_path)
end

%% Configuration Flags
% Trajectory Config
use_waypoints = false;          % (Only usable in 3D) uses the waypoints to recreate the obstacle avoidance
use_rotation_z = false;         % Rotates in the Z axis the final position
use_obstacles = false;          % (Only usable when live CBF is activated) Represents visually the obstacles
moveNdim = 3;                   % 1 -> Movement in X; 2 -> Movement in X and Y; 3 -> Movement in 3D

% Control Barrier Funtcions Config
force_CBF = false;              % Gives an arbitrary value to A or b in CBF to force the CBF condition
use_real_CBF = true;            % Applies the real conditions of CBF to evaluate the system


%%%%%%%% Plots %%%%%%%%
% Object Plot
plot_Tetramesh = false;         % Plots the tetrahedral mesh of the object

% Simulation Real-time Plot
plot_LiveVonMises = false;      % Opens a GUI plotting the stress evolution during the simulation
plot_LiveCBF = true;            % Opens a GUI plotting the simulation evolution

% Control Results Plot
plot_ControlOutputs_3D = false; % Plots the Controllers individual outputs
plot_CResults_3D = true;        % Plots the control results of the standard 3D controller (trajectory, individual positions, individual velocities, velocity norms, control errors)
plot_CResults_CBF = true;       % Plots the control results of the CBF 3D controller (trajectory, individual positions, individual velocities, velocity norms, control errors)

% Specific CBF Plots
plot_CBF_Conditions = true;     % Plots the conditions of the CBF 3D controller (Au vs b)
plot_Hs_CBF = true;             % Plots the h funtion values (in this case h = (γ x σ_yield - σ_current)
plot_GradsH_CBF = true;         % Plots the gradient of h values (in this case grad_h = (δ x σ_pred)/(2 x σ_current)

% Specific Vel CBF Plots
plot_CBF_VelCorr = true;        % Plots the correlation between the proposed velocity (the one from the 3D controller) and the real velocity applied (the one of the CBF)
plot_CBF_VelDiff = false;       % Plots the difference (prop_vs - vs) between the proposed velocity and the applied velocity
plot_VelComp = true;            % Plots the comparison between the norm of the proposed velocity, the applied velocity and the real 3D controller velocity

% Specific Stress Plots
plot_VonMisesMax = true;        % Plots the Maximum values of Von Mises recorded in each instance
plot_VMStressDiff = true;       % Plots the Difference between the real Von Mises Stress and the Predicted Von Mises Stress
plot_VMStressComp = true;       % Plots the comparison between the yield stress (σ_yield), the 3D controller stress (σ_3D), the predicted stress (σ_pred) and the real current stress (σ_curr)

% Saving plots
save_plots = false;             % Determines wether to save the plots or not

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Parameters
% Standard Parameters
N = 8;                          % Number of robots
ndims = 3;                      % Number of dimensions

% Time parameters - The simulated time in seconds will be nit*dt
dt = 0.01;                      % Time step of the simulation (s)

% Number of iterations of the control loop
switch moveNdim
    case 1
        niters = 300;
    case 2
        niters = 500;
    otherwise
        niters = 700;
end

if use_waypoints 
    niters = 1600;
end

% Physical parameters
E = 3000;                       % Young Modulus (Pa)
nu = 0.01;                      % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10;                      % External forces stiffness (N/m)
yield_stress = 210e2;           % Yield Stress (Pa)
SF = 2.5;                       % Safety Factor (Unitless)
u_sat = 1.5;                    % Velocity Saturation Value (m/s)

% Load the Precomputed Jacobian Matrix
% load('.\Experiments\Forces\MatlabInteractiveTests\JacobianUnitaryCube.mat');
load('.\Experiments\Forces\MatlabInteractiveTests\JacobianProgramaticallyTest.mat');
clear agent_actions mesh_model n_experiments particle_displacements vonMisesValues;

% Matrices for control formulation
K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % Centering Matrix (3*Na x 3*Na)
S = [0 -1; 1 0];
T = kron(eye(N),S);

% Viewing angles for the 3D plot
view_1 = 14.1759;
view_2 = 18.4354;

%% Control Gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gain values for the control
k_H = 50;        % for moving toward shape-preserving transformation
k_G = 5;         % for moving toward a configuration consistent with our deformation modes
k_c = 1;         % for centroid translation
k_s = 50;        % for scaling
k_Hd = 1.5;      % for scaling and orientation
kCBF = 1;        % for controllig the stress avoidance influence
cbf_alpha = 1;   % for controlling the risks taken by the barrier function

% Target formation scale and rotation
sd = 1;          % Target Scale (1 = no change)
thd = 0;         % Target Rotation (0 = no change)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Object Mesh

nx = 5; ny = 5; nz = 5;         % Grid nodes in x, y, z directions
sx = 1; sy = 1; sz = 1;         % Object dimensions (m)
scale_ob = 1;                   % Object scale factor

omesh = create_mesh3D(nx, ny, nz, sx * scale_ob, sy * scale_ob, sz * scale_ob); % mesh of the object: its shape and the links between nodes
omesh.shape = omesh.shape + [0, 0, -1*(min(omesh.shape(:,3))<0)*min(omesh.shape(:,3))]; % We put the object over the Z axis (which means that the lowest part of the body will be at 0 or over, regarding the Z axis, so it does not go under the floor

% Object Plot
if plot_Tetramesh
    figure;
    tetramesh(omesh.elements, omesh.shape);
end

%% Initial Configurations
% Initial node positions
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
th0_x = 0*pi/180;
th0_y = 0*pi/180;
th0_z = 0*pi/180;

% We define the initial configuration
p_init = reshape(p_init', [3*N, 1]); % (3*Na x 1)
p00 = K * p_init; % (3*Na x 1)
R0 = eul2rotm([th0_x th0_y th0_z], 'XYZ');
R0 = kron(eye(N), R0);
p0 = (s0 * R0 * p00) + reshape(g0 * ones(1,N), [3*N,1]); % (3*Na x 1)

% We define the reference configuration, c
c = K * p00; % We want to preserve its shape
c = K * c; % Reference Configuration with centroid at (0,0) % (3*Na x 1)

% Initial positions of the robots
p_3D = p0;      % p matrix 3Nx1
p_cbf = p0;     % p_cbf matrix 3Nx1

%% Desired Configurations
% Desired Scale Factor
sd0 = 1; % No Change

% Desired Rotation Angles
thd0_x = 0*pi/180;    % 0 deg
thd0_y = 0*pi/180;    % 0 deg
if use_rotation_z
    thd0_z = -180*pi/180; % Rotation of 180 deg in the Z axis
else
    thd0_z = 0*pi/180;
end

% We define the desired initial configuration
Rd0 = eul2rotm([thd0_x thd0_y thd0_z], 'XYZ');
Rd0 = kron(eye(N), Rd0);
pd = (sd0 * Rd0 * c); % Desired Initial Configuration


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


% Rotation Matrix Desired Configuration
Rd = eul2rotm([thd thd thd], 'XYZ');
Rd_T = kron(eye(N), Rd);
PT = (sd * Rd_T * pd) + reshape(gd * ones(1,N), [3*N,1]);     % final configuration with centroid in gd (3*Na x 1)

% Control terms are calculated with c. As the code is defined here, we can
% replace c with pd in the control terms code or do the following:
c = K * PT; % Desired centroid config (3*Na x 1)


%% Waypoints definition

if use_waypoints
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
if use_obstacles
    % Internal wall
    r1 = 0.2;
    center1 = g0 + [-1.35, -1.2, -0.15];
    th1 = linspace(0, 2*pi, 20);
    obstacles1 = zeros(3, length(th1));
    for i = 1:length(th1)
        obstacles1(1,i) = center1(1) + r1 * cos(th1(i));
        obstacles1(2,i) = center1(2) + r1 * sin(th1(i));
        obstacles1(3,i) = center1(3);
    end
    % External wall
    r2 = 2.5;
    center2 = g0 + [-1.35, -1.2, -0.15];
    th2 = linspace(0, 2*pi, 40);
    obstacles2 = zeros(3, length(th2));
    for i = 1:length(th2)
        obstacles2(1,i) = center2(1) + r2 * cos(th2(i));
        obstacles2(2,i) = center2(2) + r2 * sin(th2(i));
        obstacles2(3,i) = center2(3);
    end
end          

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Storage Arrays Initialization
% Standard 3D Controller
ps_3D = [];             % Positions of the standard 3D controller
gammas_H_3D = [];       % Cost relative to shape-preserving transformation
gammas_G_3D = [];       % Cost relative to configuration consistent with our deformation modes
egs_3D = [];            % Centroid errors
ess_3D = [];            % Scale errors
eths_3D = [];           % Rotation eror
eth_xs_3D = [];         % X angle errors
eth_ys_3D = [];         % Y angle errors
eth_zs_3D = [];         % Z angle errors
vs_3D = [];             % Linear velocities
nvs_3D = [];            % Linear velocities norm

% Control Results
u_Hs = [];              % Shape-preserving control (UH)
u_Gs = [];              % Deformation Control (UG)
U_ss = [];              % Scale Control (US)
u_cs = [];              % Position Control (Uc)
U_Hds = [];             % Scale and Orientation Control (UHd)
u_cbfs = [];            % Control Barrier Functions
us = [];                % Final Control Outputs (U_f)

% CBF
ps_cbf = [];            % Positions with the barrier function
prop_nvs_cbf = [];      % Desired linear velocities norm CBF (Before the CBF)
prop_vs_cbf = [];       % Desired linear velocities (Befor the CBF)
nvs_cbf = [];           % Linear velocities norm CBF
vs_cbf = [];            % Linear velocities
As = [];                % Left side of the optimization condition
bs = [];                % Right side of the optimization condition
gammas_H_cbf = [];      % Cost relative to shape-preserving transformation
gammas_G_cbf = [];      % Cost relative to configuration consistent with our deformation modes
egs_cbf = [];           % Centroid errors
ess_cbf = [];           % Scale errors
eths_cbf = [];          % Rotation eror
eth_xs_cbf = [];        % X angle errors
eth_ys_cbf = [];        % Y angle errors
eth_zs_cbf = [];        % Z angle errors
grads_hs = [];          % Gradients of the h function of the cbf
opt_hs = [];            % Record of the h function of the cbf

% Stress Related things
% CBF
vm_stresses_cbf = [];   % Von Mises stresses CBF control
u_hats_cbf = [];        % Record of the element displacements on the CBF control
element_disps_cbf = []; % Record of the element displacements predicted on the CBF control
tensor_stresses_cbf= [];% Stress tensors CBF control
pred_strains_cbf = [];  % Record of the predicted strains produced on the cbf control
strains_cbf = [];       % Record of the strains produced on the cbf control
elem_stresses_cbf = []; % Record of the element stresses produced on the cbf control
pred_stresses_cbf = []; % Record of the predicted stress tensors produced on the cbf control
pred_vmStresses_cbf =[];% Record of the predicted von Mises stresses produced on the cbf control
scaled_deltas_cbf = []; % Record of the von Mises derivative produced on the cbf control
pred_elem_stresses_cbf = [];% Record of the predicted element stress tensors produced on the cbf control
% 3D Controller
u_hats_3D = [];         % Record of the element displacements on the 3D control
strains_3D = [];        % Record of the strains produced on the 3D control
element_disps_3D = [];  % Record of the element displacements predicted on the 3D control
vm_stresses_3D = [];    % Record of the nodal von Mises stress of the 3D controller
tensors_stresses_3D =[];% Record of the nodal stress tensors of the 3D controller
scaled_deltas_3D = [];  % Record of the von Mises derivative produced on the 3D control
pred_strains_3D = [];   % Record of the predicted strains produced on the 3D control
elem_stresses_3D = [];  % Record of the element stresses produced on the 3D control
pred_stresses_3D = [];  % Record of the predicted stress tensors produced on the 3D control
pred_vmStresses_3D = [];% Record of the predicted von Mises stresses produced on the 3D control
pred_elem_stresses_3D = [];% Record of the predicted element stress tensors produced on the 3D control


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plotting Setup

plotaxfont='Helvetica';
color_robots = ["#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7", "#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7"]; % 14 robots
color_graphs = ["#0073bd", "#77a5c3", "#d9541a", "#edb120", "#7e2f8e"];

% Limits
if use_obstacles
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


% For seeing the von mises stress in real-time
if plot_LiveVonMises
    vonMises_fig = figure;
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    hold on
    box on
    view(64.3440,35.9240);
end

% For seeing the simulation in real time
if plot_LiveCBF
    trajectory_fig = figure;
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    hold on
    box on
    
    if moveNdim == 3
        xlim(x_limit/zoom_factor) % to define the bounds of the plot properly
        ylim(y_limit/zoom_factor)
        zlim([-0.5 2])
        daspect([1 1 1]) % to have same plotted size of units in x and y
        view(5.1317,25.1351);
        grid on;
    else
        xlim(x_limit) % to define the bounds of the plot properly
        ylim(y_limit)
        zlim(z_limit)
        daspect([1 1 1]) % to have same plotted size of units in x and y
    end

end


%% Animate the Connections between the agents

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

%% Plotting the initial positions of the agent
if plot_LiveCBF
    figure(trajectory_fig);
    % Initial, target and current positions
    for i = 1:N
        plot3(p0(3*i-2,1), p0(3*i-1,1), p0(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 15);
        plot3(PT(3*i-2,1), PT(3*i-1,1), PT(3*i,1), 'o', 'color', color_robots(i), 'MarkerSize', 7, 'LineWidth', 1.5);
        robot_tr(i) = plot3(p_3D(3*i-2,1), p_3D(3*i-1,1), p_3D(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 30);
        robot_path(i) = plot3(p_3D(3*i-2,:), p_3D(3*i-1,:), p_3D(3*i,:), '-', 'color', color_robots(i), 'linewidth', 2); % data.path lines
    end
    % Lines between robots
    for i = 1:size(pairs,1)
        plot3([p0(3*pairs(i,1)-2), p0(3*pairs(i,2)-2)], [p0(3*pairs(i,1)-1), p0(3*pairs(i,2)-1)], [p0(3*pairs(i,1)), p0(3*pairs(i,2))], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1.5, 'HandleVisibility', 'off');
        plot3([PT(3*pairs(i,1)-2), PT(3*pairs(i,2)-2)], [PT(3*pairs(i,1)-1), PT(3*pairs(i,2)-1)], [PT(3*pairs(i,1)), PT(3*pairs(i,2))], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off');
        robot_lines(i) = plot3([p_3D(3*pairs(i,1)-2), p_3D(3*pairs(i,2)-2)], [p_3D(3*pairs(i,1)-1), p_3D(3*pairs(i,2)-1)], [p_3D(3*pairs(i,1)), p_3D(3*pairs(i,2))], '--', 'color', [0.5 0.8 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off');
    end
    plot3(g0(1,:), g0(2,:), g0(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    % plot3(gd(1,:), gd(2,:), gd(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    
    if use_waypoints
        plot3(waypoints_g(1,:), waypoints_g(2,:), waypoints_g(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    end

    if use_obstacles
        for i = 1:size(pairs,1)
            object_lines(i) = plot3([p_3D(3*pairs(i,1)-2), p_3D(3*pairs(i,2)-2)], [p_3D(3*pairs(i,1)-1), p_3D(3*pairs(i,2)-1)], [p_3D(3*pairs(i,1)), p_3D(3*pairs(i,2))], 'k-', 'linewidth', 0.5);
        end
        
        % Obtacles plotting
        for i = 0:50
            plot3(obstacles1(1,:), obstacles1(2,:), obstacles1(3,:) + i*0.02, 'k-', 'LineWidth', 2);
        end
        for i = 0:10
            plot3(obstacles2(1,:), obstacles2(2,:), obstacles2(3,:) + i*0.02, 'k-', 'LineWidth', 2);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Object Simulation
handled_nodes = [];
for i = 0:N-1
    differences = omesh.shape - p_init(3*i+1:3*i+3)';       % Subtract target from each column
    dists = sqrt(sum(differences.^2, 2)); % Euclidean distances for each column
    [~, idx] = min(dists);          % minDist is the minimum distance, idx is the column index
    handled_nodes = [handled_nodes, idx];
end

n_iters_arap = 3; % choose a value of 2 or 3: the higher the value the stiffer the simulated object

arap_params = create_params_for_arap(omesh, handled_nodes, [], n_iters_arap);  % Create model parameters

% Simulate the initial position of the object's nodes
Pob_3D = simulate_object_arap(arap_params, omesh.shape', [reshape(p_3D, [3,N])]); % Object Nodes Position (3xNp)
Pob_cbf = Pob_3D; % CBF Object Nodes Position

%% Object plotting initialization
if use_obstacles == 1
    for i = 1:size(arap_params.edg, 1)   % we plot the edges of the sheet's mesh between nodes
        object_mesh(i) = plot3([Pob_3D(1,arap_params.edg(i,1)) Pob_3D(1,arap_params.edg(i,2))], ...
                     [Pob_3D(2,arap_params.edg(i,1)) Pob_3D(2,arap_params.edg(i,2))], ...
                     [Pob_3D(3,arap_params.edg(i,1)) Pob_3D(3,arap_params.edg(i,2))], ...
                     '-', 'color', [0.5 0.5 0.8], 'linewidth', 1);
    end
                       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Main Simulation loop
for it_loop = 1:niters
    % Store current positions
    ps_3D = [ps_3D, p_3D]; % Control 3D (3*Na x N_iters)
    ps_cbf = [ps_cbf, p_cbf]; % CBF (3*Na x N_iters)
    
    % Compute centroid
    g = (1/N) * [sum(p_3D(1:3:end)); sum(p_3D(2:3:end)); sum(p_3D(3:3:end))]; % (3x1)

    % Simulate object deformation
    Pob_3D = simulate_object_arap(arap_params, Pob_3D, [reshape(p_3D, [3,N])]);
    Pob_cbf = simulate_object_arap(arap_params, Pob_cbf, [reshape(p_cbf, [3,N])]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Rotate view if obstacles present
    if (it_loop > 25) && (it_loop < 550) && use_obstacles
        view(view_1-(it_loop-25)*0.5, view_2)
    end
    
    % Update live plot
    if plot_LiveCBF
        try
            figure(trajectory_fig);
            % Plot the robots in their current position in each iteration
            for i = 1:N
                % Current Position
                robot_tr(i).XData = p_cbf(3*i-2,1);
                robot_tr(i).YData = p_cbf(3*i-1,1);
                robot_tr(i).ZData = p_cbf(3*i,1);

                % Path
                robot_path(i).XData = ps_cbf(3*i-2,:);
                robot_path(i).YData = ps_cbf(3*i-1,:);
                robot_path(i).ZData = ps_cbf(3*i,:);
            end
            for i = 1:size(pairs,1)
                robot_lines(i).XData = [p_cbf(3*pairs(i,1)-2), p_cbf(3*pairs(i,2)-2)];
                robot_lines(i).YData = [p_cbf(3*pairs(i,1)-1), p_cbf(3*pairs(i,2)-1)];
                robot_lines(i).ZData = [p_cbf(3*pairs(i,1)), p_cbf(3*pairs(i,2))];
            end

            if use_obstacles
                for i = 1:size(object_lines,2)
                    object_lines(i).XData = [p_3D(3*pairs(i,1)-2), p_3D(3*pairs(i,2)-2)];
                    object_lines(i).YData = [p_3D(3*pairs(i,1)-1), p_3D(3*pairs(i,2)-1)];
                    object_lines(i).ZData = [p_3D(3*pairs(i,1)), p_3D(3*pairs(i,2))];
                end
            end
        catch
            disp("Trajectory Live Window Closed");
            plot_LiveCBF = false;
        end
    end


    if use_obstacles == 1
    
            for i = 1:size(arap_params.edg, 1)
                object_mesh(i).XData = [Pob_3D(1,arap_params.edg(i,1)) Pob_3D(1,arap_params.edg(i,2))];
                object_mesh(i).YData = [Pob_3D(2,arap_params.edg(i,1)) Pob_3D(2,arap_params.edg(i,2))];
                object_mesh(i).ZData = [Pob_3D(3,arap_params.edg(i,1)) Pob_3D(3,arap_params.edg(i,2))];
            end
                
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if use_waypoints
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
        
    if use_real_CBF
        %%%%%%%%%% - CBF CONTROL - %%%%%%%%%%
        [~, ~, u_3D, ~, ~, ~, ~, ~, u_cbf, agent_cbf_positions, ~, ~, ~, ~, ~, ~, ~, vm_stress, stress_tensor, curr_elem_stress, curr_strain, A, b, grad_h, scaled_delta, pred_nodal_stresses, pred_stresses_element, pred_strains, element_displacements, u_hat_cbf, Le_cbf, Ce_cbf] = ForceControl3D_Debug(Pob0', Pob_cbf', reshape(p_cbf, [ndims,N]), agent_destinations, omesh.elements, J, E, nu, yield_stress, SF, kCBF, cbf_alpha, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
        %%%%%%%%%% - CBF CONTROL - %%%%%%%%%%

        % Displacements
        u_hats_cbf = cat(3, u_hats_cbf, squeeze(u_hat_cbf)); % (12xN_tetxNiter)
        element_disps_cbf = cat(3, element_disps_cbf, reshape(element_displacements*u_3D,[12,size(u_hat_cbf,3)]));

        % Current Stresses
        vm_stresses_cbf = [vm_stresses_cbf, vm_stress];
        tensor_stresses_cbf = cat(3, tensor_stresses_cbf, stress_tensor);
        elem_stresses_cbf = cat(3, elem_stresses_cbf, squeeze(curr_elem_stress));
        
        % Strains recording
        pred_strains_cbf = cat(3, pred_strains_cbf, reshape(pred_strains*u_3D,[6,size(omesh.elements,1)]));
        % pred_strains_cbf = cat(3, pred_strains_cbf, pred_strains*u_cbf);
        strains_cbf = cat(3, strains_cbf, squeeze(curr_strain));

        % von Mises Derivative
        scaled_deltas_cbf = cat(3, scaled_deltas_cbf, scaled_delta);

        % Predicted Stresses
        pred_nodal_stresses = pred_nodal_stresses*u_3D;
        % pred_nodal_stresses = pred_nodal_stresses*u_cbf;
        pred_nodal_stresses = reshape(pred_nodal_stresses,[6,size(omesh.shape,1)]);
        pred_stresses_cbf = cat(3, pred_stresses_cbf, pred_nodal_stresses); % Per node
        
        pred_element_stress_cbf = reshape(pred_stresses_element*u_3D,[6,size(omesh.elements,1)]);
        pred_elem_stresses_cbf = cat(3, pred_elem_stresses_cbf, pred_element_stress_cbf); % per element
        % Predicted von Mises Stress
        [pred_vm_stress_cbf,~] = VonMisesStressComp(pred_element_stress_cbf, omesh.elements, size(omesh.shape,1));
        pred_vmStresses_cbf = [pred_vmStresses_cbf, pred_vm_stress_cbf];

        % Gradients of h
        grads_hs = [grads_hs, grad_h*u_3D];
        % grads_hs = [grads_hs, grad_h*u_cbf];


    end

    % [u, U_f, U_H, u_G, U_s, u_c, U_Hd, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual] = TransportationControl3D_Debug(reshape(p, [ndims,N]), agent_destinations, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
    [~, U_f, u, U_H, u_G, U_s, u_c, U_Hd, u_3D_cbf, agent_positions_3D, agent_destinations_3D, gamma_H_3D, gamma_G_3D, eg_3D, es_3D, eth_3D, eth_individual_3D, curr_vmSigma_3D, curr_SigmaTensor_3D, curr_elem_stress_3D, curr_strain_3D, ~, ~, ~, scaled_delta_3D, pred_nodal_stresses_3D, pred_stresses_element_3D, pred_strain_3D, element_displacements_3D, u_hat_3D, Le_3D, Ce_3D] = ForceControl3D_Debug(Pob0', Pob_3D', reshape(p_3D, [ndims,N]), agent_destinations, omesh.elements, J, E, nu, yield_stress, SF, kCBF, cbf_alpha, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);

    % Displacements
    u_hats_3D = cat(3, u_hats_3D, squeeze(u_hat_3D)); % (12xN_tetxNiter)
    element_disps_3D = cat(3, element_disps_3D, reshape(element_displacements_3D*u,[12,size(u_hat_3D,3)]));

    % Current Stresses
    vm_stresses_3D = [vm_stresses_3D, curr_vmSigma_3D];
    tensors_stresses_3D = cat(3, tensors_stresses_3D, curr_SigmaTensor_3D);
    elem_stresses_3D = cat(3,elem_stresses_3D, squeeze(curr_elem_stress_3D));
    
    % Strains recording
    pred_strains_3D = cat(3, pred_strains_3D, reshape(pred_strain_3D*u,[6,size(omesh.elements,1)]));
    % pred_strains_cbf = cat(3, pred_strains_cbf, pred_strains*u_cbf);
    strains_3D = cat(3, strains_3D, squeeze(curr_strain_3D));

    % von Mises Derivative
    scaled_deltas_3D = cat(3, scaled_deltas_3D, scaled_delta_3D);

    % Predicted Stresses
    pred_nodal_stresses_3D = pred_nodal_stresses_3D*u;
    % pred_nodal_stresses = pred_nodal_stresses*u_cbf;
    pred_nodal_stresses_3D = reshape(pred_nodal_stresses_3D,[6,size(omesh.shape,1)]);
    pred_stresses_3D = cat(3, pred_stresses_3D, pred_nodal_stresses_3D); % Per node

    pred_element_stress_3D = reshape(pred_stresses_element_3D*u,[6,size(omesh.elements,1)]);
    pred_elem_stresses_3D = cat(3, pred_elem_stresses_3D, pred_element_stress_3D); % per element
    % Predicted von Mises Stress
    [pred_vm_stress_3D,~] = VonMisesStressComp(pred_element_stress_3D, omesh.elements, size(omesh.shape,1));
    pred_vmStresses_3D = [pred_vmStresses_3D, pred_vm_stress_3D];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% CBF

    % Compute the MaxStress
    MaxStress = yield_stress/SF;

    H = 2 * eye(3*N);
    f = -2 * u(:);

    if ~use_real_CBF
        if it_loop > niters/2 && force_CBF
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
    gammas_H_3D = [gammas_H_3D; gamma_H_3D];
    gammas_G_3D = [gammas_G_3D; gamma_G_3D];
    egs_3D = [egs_3D; eg_3D];                                % Centroid error
    ess_3D = [ess_3D; es_3D];                                % Scale error

    eths_3D = [eths_3D; eth_3D];                             % Rotation error
    eth_xs_3D = [eth_xs_3D; eth_individual_3D(1)];           % Roll Error
    eth_ys_3D = [eth_ys_3D; eth_individual_3D(2)];           % Pitch Error
    eth_zs_3D = [eth_zs_3D; eth_individual_3D(3)];           % Yaw Error

    u_Hs = cat(3, u_Hs, U_H);                       % shape-preserving control (UH)
    u_Gs = cat(3, u_Gs, reshape(u_G,[3,N]));        % Deformation Control (UG)
    U_ss = cat(3, U_ss, U_s);                       % Scale Control (US)
    u_cs = cat(3, u_cs, reshape(u_c,[3,N]));        % Position Control (Uc)
    U_Hds = cat(3, U_Hds, U_Hd);                    % Scale and Orientation Control (UHd)
    u_cbfs = cat(3, u_cbfs, reshape(u_cbf,[3,N]));  % Control Barrier Functions
    us = cat(3, us, reshape(u,[3,N]));              % Standard Final Output


    %%%%%%%%%%%%%%%%%%%%%%% 
    
    u = u(:);

    p_3D = p_3D + u*dt; % apply control input
    p_cbf = p_cbf + u_cbf*dt;
    
    % Velocity norm
    for i = 1:N 
        nv(i,1) = norm(u(3*i-2:3*i,1)); % we store the norms of velocity
        nv_cbf(i, 1) = norm(u_cbf(3*i-2:3*i,1));
        if use_real_CBF
            prop_nv_cbf(i,1) = norm(u_3D(3*i-2:3*i,1));
        end
    end
    
    % Store current velocity data
    vs_3D = [vs_3D u];
    nvs_3D = [nvs_3D nv];

    vs_cbf = [vs_cbf u_cbf];
    nvs_cbf = [nvs_cbf nv_cbf];

    % Proposed 3D vel
    if use_real_CBF
        prop_vs_cbf = [prop_vs_cbf u_3D];
        prop_nvs_cbf = [prop_nvs_cbf prop_nv_cbf];
    end

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
    

    if plot_LiveVonMises
        figure(vonMises_fig);
        trisurf(omesh.elements, Pob0(1,:)', Pob0(2,:)', Pob0(3,:)', vm_stress, 'EdgeColor', 'none');
    end

    
    %% Pause for live plot
    if plot_LiveCBF
        pause(0.005) % we give some time for the user to visualize the created plot
    end
 
end

%% Computation of the Stress-related things
if use_real_CBF

    % Computation of the predicted VM stress
    pred_sigma_12 = pred_stresses_cbf(1,:,:) - pred_stresses_cbf(2,:,:); % sigma_x - sigma_y
    pred_sigma_23 = pred_stresses_cbf(2,:,:) - pred_stresses_cbf(3,:,:); % sigma_y - sigma_z
    pred_sigma_31 = pred_stresses_cbf(3,:,:) - pred_stresses_cbf(1,:,:); % sigma_z - sigma_x
    pred_vmStress(:,:) = sqrt(0.5 .* (pred_sigma_12.^2 + pred_sigma_23.^2 + pred_sigma_31.^2) + ...
        3 .* (pred_stresses_cbf(4,:,:).^2 + pred_stresses_cbf(5,:,:).^2 + pred_stresses_cbf(6,:,:).^2));
    
    % Difference between von mises stresses based on the predicted nodal
    % stresses
    vonMises_diffs = pred_vmStress - vm_stress;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot control outputs
if plot_ControlOutputs_3D
    plot_control_outputs;
end

%% Plot standard data
SimData2Struct_3DControl;
if plot_CResults_3D
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

if plot_CResults_CBF
    plotControlResults(data_cbf);
end

%% Plot the CBF diferences -----> Au VS b
if plot_CBF_Conditions
    figure;
   % Most representative values comparison
    % b
    plot(max(bs),'DisplayName', "b_m_a_x", 'Color','r', 'LineWidth', 2, 'LineStyle',':');
    hold on;
    plot(min(bs),'DisplayName', "b_m_i_n", 'Color','b', 'LineWidth', 2, 'LineStyle',':');
    plot(mean(bs),'DisplayName', "b_m_e_a_n", 'Color', 'black', 'LineWidth', 2, 'LineStyle',':');

    % grad_h
    plot(max(As),'DisplayName', "(Au)_m_a_x", 'Color','r', 'LineWidth', 2);
    plot(min(As),'DisplayName', "(Au)_m_i_n", 'Color','b', 'LineWidth', 2);
    plot(mean(As),'DisplayName', "(Au)_m_e_a_n", 'Color', 'black', 'LineWidth', 2);
    grid on;
    hold off;
    title("A x u_c_b_f <= b  || gradH <= α x h");
    xlabel("Iters");
    ylabel("unitless");
    grid on;
    legend show;
end

%% Plot h Function Values
max_stresses = ones(size(vm_stresses_cbf))*(yield_stress/SF);
hs = max_stresses - vm_stresses_cbf;
if plot_Hs_CBF

    % Full values plot
    figure;
    plot(hs);
    title('CBF h - (σ_m_a_x - σ_c_u_r_r)');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

    % Most representative values plot
    figure;
    plot(max(hs),'DisplayName', "max", 'Color','r', 'LineWidth', 2);
    hold on;
    plot(min(hs),'DisplayName', "min", 'Color','b', 'LineWidth', 2);
    plot(mean(hs),'DisplayName', "mean", 'Color', 'black', 'LineWidth', 2, 'LineStyle',':');
    legend show;
    title('CBF h - (σ_m_a_x - σ_c_u_r_r)');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

    
end


%% Plot Gradient of h
if plot_GradsH_CBF
    % Full Values plot
    figure;
    plot(grads_hs);
    title('Grad_h - Au_c_b_f');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

    % Most representative values comparison with h
    figure;
    % H
    plot(max(hs),'DisplayName', "h_m_a_x", 'Color','r', 'LineWidth', 2, 'LineStyle',':');
    hold on;
    plot(min(hs),'DisplayName', "h_m_i_n", 'Color','b', 'LineWidth', 2, 'LineStyle',':');
    plot(mean(hs),'DisplayName', "h_m_e_a_n", 'Color', 'black', 'LineWidth', 2, 'LineStyle',':');

    % grad_h
    plot(max(grads_hs),'DisplayName', "gradh_m_a_x", 'Color','r', 'LineWidth', 2);
    plot(min(grads_hs),'DisplayName', "gradh_m_i_n", 'Color','b', 'LineWidth', 2);
    plot(mean(grads_hs),'DisplayName', "gradh_m_e_a_n", 'Color', 'black', 'LineWidth', 2);
    legend show;
    title('CBF h VS grad_h');
    xlabel('Iteration');
    ylabel('VM Stress (Pa)');
    grid on;

end

%% Individual Velocity Correction plot
if plot_CBF_VelCorr
    % LINEAR VELOCITY IN X
    % We create the figure and set the parameters
    figure
    % subplot(3,1,1);
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        plot(prop_vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
        % plot(vs(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity in X (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity X Comp', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY IN Y
    % We create the figure and set the parameters
    % subplot(3,1,2);
    figure;
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        plot(prop_vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
        % plot(vs(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity in Y (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Y Comp', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY IN Z
    % We create the figure and set the parameters
    % subplot(3,1,3);
    figure
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        plot(prop_vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
        % plot(vs(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity in Z (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Z Comp', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY NORM
    % We create the figure and set the parameters
    figure
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    
    % Representation
    for i = 1:N
        plot(nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
        plot(prop_nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Des vel Agent %i',i), 'LineStyle','--');
        % plot(nvs(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('Standard vel Agent %i',i), 'LineStyle',':');
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity norm (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Norm Comp', 'FontSize', 14);
    hold off;

end

%% Velocity Differences

if plot_CBF_VelDiff
    % LINEAR VELOCITY IN X
    % We create the figure and set the parameters
    figure
    subplot(3,1,1);
    % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(prop_vs_cbf(3*i-2,:) - vs_cbf(3*i-2,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity diff in X (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity X Difference', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY IN Y
    % We create the figure and set the parameters
    subplot(3,1,2);
    % figure;
    % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(prop_vs_cbf(3*i-1,:) - vs_cbf(3*i-1,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity diff in Y (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    % legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Y Difference', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY IN Z
    % We create the figure and set the parameters
    subplot(3,1,3);
    % figure
    % set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    grid on
    
    % Representation
    for i = 1:N
        plot(prop_vs_cbf(3*i,:) - vs_cbf(3*i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity diff in Z (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    % legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Z Difference', 'FontSize', 14);
    hold off;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % LINEAR VELOCITY NORM
    % We create the figure and set the parameters
    figure
    set(gcf, 'Position',  [300, 50, 1000, 650], 'color','w')
    box on
    hold on
    
    % Representation
    for i = 1:N
        plot(prop_nvs_cbf(i,:) - nvs_cbf(i,:), 'color', color_robots(i), 'linewidth', 2, 'DisplayName', sprintf('CBF vel Agent %i',i));
    end
    
    % Titles and dimensions
    set(gca, 'FontSize', 16, 'FontName', plotaxfont); 
    xlabel('Time (s)', 'FontSize', 18)
    ylabel('Linear velocity norm (m/s)', 'FontSize', 18)
    
    xlim([0, niters]);
    
    % legend
    legend('FontSize', 14, 'Location', 'NorthOutside', 'NumColumns', 4, 'Box', 'off')
    
    % Title
    title('Linear Velocity Norm Difference', 'FontSize', 14);
    hold off;

end

%% Velocity comparation

if plot_VelComp

    figure
    plot(nvs_3D(1,:)');
    hold on;
    plot(nvs_cbf(1,:)');
    if use_real_CBF
        plot(prop_nvs_cbf(1,:)');
    end
    hold off;
    title("Velocity Norm Comparison");
    xlabel("Iters");
    ylabel("m/s");
    % legend(["Agend 1 - std", "Agend 2 - std", "Agend 3 - std", "Agend 4 - std", "Agend 1 - cbf", "Agend 2 - cbf", "Agend 3 - cbf", "Agend 4 - cbf"]);
    if use_real_CBF
        legend(["Agend 1 - std", "Agend 1 - cbf", "Agent 1 - Proposed cbf"]);
    else
        legend(["Agend 1 - std", "Agend 1 - cbf"]);
    end

end

%% Plot von Mises maximum ------> Curr VM
if plot_VonMisesMax
    figure;
    hold on;
    for a = 1:N
        plot(max(vm_stresses_cbf),'Color','b');
    end
    hold off;
    title("Maximum Von Mises per iteration");
    xlabel("Iters");
    ylabel("von Mises Stress (PA)");
    grid on;
    legend("MaxVonMises");

    % % Cube plot
    % VMEvolFig = figure;
    % h = trisurf(omesh.elements, Pob0(1,:)', Pob0(2,:)', Pob0(3,:)', 'EdgeColor', 'none');
    % colormap(jet); colorbar; caxis([min(vm_stresses(:)) max(vm_stresses(:))]);
    % figure(VMEvolFig);
    % 
    % for iter = 1:niters
    % 
    %     h.CData = vm_stresses(:,iter);
    %     title(sprintf('VM Stress Evolution - Iteration %d', iter));
    %     drawnow;
    %     pause(0.25); % Adjust speed
    % end
end


%% Plot Stress Differences

if plot_VMStressDiff

    % Reshape data: (nodes x iterations)
    vm_diff_matrix = reshape(vonMises_diffs, [], niters);
    
    % 3D plot
    [NodeGrid, IterGrid] = meshgrid(1:size(vonMises_diffs,1), 1:niters);
    
    figure;
    surf(IterGrid, NodeGrid, vm_diff_matrix');
    xlabel('Iteration');
    ylabel('Node Index');
    zlabel('Stress Difference (Pa)');
    title('3D Stress Difference Surface - (σ_c_u_r_r VS σ_p_r_e_d)');
    shading interp;

    % Max over Min
    max_diff = squeeze(max(vonMises_diffs, [], 1));
    min_diff = squeeze(min(vonMises_diffs, [], 1));
    mean_diff = squeeze(mean(vonMises_diffs, 1));
    
    figure;
    plot(max_diff, 'r', 'LineWidth', 2); hold on;
    plot(min_diff, 'b', 'LineWidth', 2);
    plot(mean_diff, 'k--', 'LineWidth', 2);
    xlabel('Iteration');
    ylabel('Stress Difference (Pa)');
    legend('Max', 'Min', 'Mean');
    title('Aggregate Stress Differences Predicted vs Real Over Time');

    % % Cube plot
    % VMDIFEvolFig = figure;
    % h = trisurf(omesh.elements, Pob0(1,:)', Pob0(2,:)', Pob0(3,:)', 'EdgeColor', 'none');
    % colormap(jet); colorbar; caxis([min(vonMises_diffs(:)) max(vonMises_diffs(:))]);
    % 
    % for iter = 1:niters
    %     h.CData = vonMises_diffs(:,iter);
    %     title(sprintf('Current VS Predicted - Iteration %d', iter));
    %     drawnow;
    %     pause(0.1); % Adjust speed
    % end

end

%% Plot VM Stress Comparison
if plot_VMStressComp
    % This plot the differences between the maximum value of the von Mises
    % stress during the simulation
    figure;
    plot(max(vm_stresses_cbf),'Color','b','DisplayName', "CBF - Max VMStress (σ_c_u_r_r)", 'linewidth', 2);
    hold on;
    plot(repelem(yield_stress/SF,size(vm_stresses_cbf,2)),'Color', 'r', 'DisplayName', "Yield Stress (σ_y_i_e_l_d)", 'linewidth', 2);
    plot(max(pred_vmStress),'Color','g','DisplayName',"CBF - Max Pred VMStres (σ_p_r_e_d)", 'linewidth', 2);
    plot(max(vm_stresses_3D),'Color',[1 0.5 0],'DisplayName',"3D Control - Max VMStress (σ_3_D)", 'linewidth',2);
    hold off;
    xlabel('Iteration');
    ylabel('VM Stress');
    title('Von Mises Stress Comparison');
    legend show;
    grid on;
end
