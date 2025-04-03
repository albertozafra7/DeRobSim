%% Planar 3D control
clear;
close all;
clc; 

%% General Script configuration
% Trajectory config
waypoints = false;
rotation_z = false;
obstacle_avoidance = true;
moveNdim = 3; % 1 -> Movement in X; 2 -> Movement in X and Y; 3 -> Movement in 3D

% If we want to use the Control Barrier Functions
forceCBF = false;

% If we want to plot the control results
plotLiveCBF = false;
plotCResults = false;
plotCResultsCBF = true;
plotCBFDifferences = true;
plotCBFParams = true;
plotControlOutputs = false;
plotCBFVelCorr = true;
plotCBFVelDiff = true;
plotVelComp = true;
plotStresses = false;


%% Choice of general parameters of the simulation

N = 4; % number of robots
ndims = 3; % Number of dimensions
% We define some matrices we will use in our formulation
K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix (3*Na x 3*Na)
S = [0 -1; 1 0];
T = kron(eye(N),S);

% Time parameters - The simulated time in seconds will be nit*dt
dt = 0.01;      % time step of the simulation
if waypoints
    niters = 1600;
elseif obstacle_avoidance
    niters = 1200;
else
    niters = 700;%1600;  % number of iterations of the control loop
end

% Physical parameters
E = 3000; % Young Modulus (Pa)
nu = 0.01; % Poisson Ratio (Unitless) [-0.1,0.5]
kext = 10; % Stiffness off the external forces (N/m)
yield_stress = 210e2; % Yield Stress (Pa)
SF = 0.0005;


%% Initial Set-Up of the Agents
p_init = [ 0.5,  0.5,  0.0;... % Agent 5
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

%% Objects definition
if obstacle_avoidance
    switch moveNdim
        case 1
            obstacles.center = [-1, 0.5, 0];
        case 2
            obstacles.center = [-1, 1.5, 0];
        otherwise
            obstacles.center = [-1, 1.5, 0.5];
    end

    obstacles.radius = 0.25;
    obstacles.color = '#ff0000';
end

%% Initial conditions of the simulation and required simulation parameters

% Initial positions of the robots
p = p0;         % p matrix 3Nx1
p_cbf = p0;     % p_cbf matrix 3Nx1

% Velocity limit value
u_sat = 1.5;    % m/s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gain values for the control
k_H = 50;         % for moving toward shape-preserving transformation
k_G = 5;          % for moving toward a configuration consistent with our deformation modes
k_c = 1;          % for centroid translation
k_s = 50;         % for scaling
k_Hd = 1.5;       % for scaling and orientation
kCBF = 2;         % for controllig the stress avoidance influence
cbf_alpha = 0.15; % for controlling the risks taken by the barrier function

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
prop_nvs_cbf = [];      % desired linear velocities norm CBF (Before the CBF)
prop_vs_cbf = [];       % desired linear velocities (Befor the CBF)
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
grads_hs = [];          % gradients of the h function of the cbf
opt_hs = [];            % record of the h function of the cbf

% Object avoidance
if obstacle_avoidance
    Agent2Obst_dists = [];  % distance from agents to obstacles (1xN)
    grad_Agent2Obst = [];   % gradient of the distances from agents to obstacles (3xN)
end



%% Plotting

plotaxfont='Helvetica';
color_robots = ["#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7", "#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04", "#727272", "#26acb1", "#9769d7"]; % 14 robots
color_graphs = ["#0073bd", "#77a5c3", "#d9541a", "#edb120", "#7e2f8e"];

% Limits
if waypoints
    x_limit = [-4 1];
    y_limit = [-3 1];
    z_limit = [-1 2];
else
    x_limit = [-4 2];
    y_limit = [-1 4];
    z_limit = [0 3];
    zoom_factor = 1.25;  % greater than 1 means zoom out
    x_limit = x_limit * zoom_factor;
    y_limit = y_limit* zoom_factor;
    z_limit = z_limit * zoom_factor;
end

if plotLiveCBF
    figure
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

pairs = [pairs; [1 4]; [2 3]];
%%%

% for i = 1:size(pairs,1)
%     object_lines(i) = plot3([p(3*pairs(i,1)-2), p(3*pairs(i,2)-2)], [p(3*pairs(i,1)-1), p(3*pairs(i,2)-1)], [p(3*pairs(i,1)), p(3*pairs(i,2))], 'k-', 'linewidth', 0.5);
% end

if plotLiveCBF
    % Initial, target and current positions
    for i = 1:N
        plot3(p0(3*i-2,1), p0(3*i-1,1), p0(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 15);
        plot3(PT(3*i-2,1), PT(3*i-1,1), PT(3*i,1), 'o', 'color', color_robots(i), 'MarkerSize', 7, 'LineWidth', 1.5);
        robot_tr(i) = plot3(p(3*i-2,1), p(3*i-1,1), p(3*i,1), '.', 'color', color_robots(i), 'MarkerSize', 30);
    end
    % Lines between robots
    for i = 1:size(pairs,1)
        plot3([p0(3*pairs(i,1)-2), p0(3*pairs(i,2)-2)], [p0(3*pairs(i,1)-1), p0(3*pairs(i,2)-1)], [p0(3*pairs(i,1)), p0(3*pairs(i,2))], '--', 'color', [0.5 0.5 0.8], 'linewidth', 1.5, 'HandleVisibility', 'off');
        plot3([PT(3*pairs(i,1)-2), PT(3*pairs(i,2)-2)], [PT(3*pairs(i,1)-1), PT(3*pairs(i,2)-1)], [PT(3*pairs(i,1)), PT(3*pairs(i,2))], '--', 'color', [0.8 0.5 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off');
        robot_lines(i) = plot3([p(3*pairs(i,1)-2), p(3*pairs(i,2)-2)], [p(3*pairs(i,1)-1), p(3*pairs(i,2)-1)], [p(3*pairs(i,1)), p(3*pairs(i,2))], '--', 'color', [0.5 0.8 0.5], 'linewidth', 1.5, 'HandleVisibility', 'off');
    end
    plot3(g0(1,:), g0(2,:), g0(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    % plot3(gd(1,:), gd(2,:), gd(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    if waypoints
        plot3(waypoints_g(1,:), waypoints_g(2,:), waypoints_g(3,:), '+', 'markersize', 5, 'color', "#9f9f9f", 'markerfacecolor', "#9f9f9f", 'linewidth', 1.2)
    end
    if obstacle_avoidance
        for o = 1:length(obstacles)
            obs = obstacles(o);
            [X,Y,Z] = sphere;
            X2 = X * obs.radius;
            Y2 = Y * obs.radius;
            Z2 = Z * obs.radius;
            surf(X2 + obs.center(1),Y2 + obs.center(2),Z2 + obs.center(3),FaceColor=[0.2,0.2,0.2],FaceAlpha=0.3);
        end
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

    if plotLiveCBF
        try
            % Plot the robots in their current position in each iteration
            for i = 1:N
                robot_tr(i).XData = p_cbf(3*i-2,1);
                robot_tr(i).YData = p_cbf(3*i-1,1);
                robot_tr(i).ZData = p_cbf(3*i,1);
            end
            for i = 1:size(pairs,1)
                robot_lines(i).XData = [p_cbf(3*pairs(i,1)-2), p_cbf(3*pairs(i,2)-2)];
                robot_lines(i).YData = [p_cbf(3*pairs(i,1)-1), p_cbf(3*pairs(i,2)-1)];
                robot_lines(i).ZData = [p_cbf(3*pairs(i,1)), p_cbf(3*pairs(i,2))];
            end
        catch
            disp("Live Window Closed");
            plotLiveCBF = false;
        end
    end

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

    %% Control Algorithm

    [u, U_f, U_H, u_G, U_s, u_c, U_Hd, agent_positions, agent_destinations, gamma_H, gamma_G, eg, es, eth, eth_individual] = TransportationControl3D_Debug(reshape(p, [ndims,N]), agent_destinations, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
    
    if obstacle_avoidance
        [u_obst, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = TransportationControl3D_Debug(reshape(p_cbf, [ndims,N]), agent_destinations, k_H, k_G, k_s, k_c, k_Hd, sd, thd, u_sat);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% CBF

    if obstacle_avoidance
        H = 2 * eye(3*N);
        f = -2 * u_obst(:);

        dist2obsts = zeros(length(N),1);
        grad_dist2obst = zeros(length(N),3);
        agent_cbf_pos = reshape(p_cbf, [ndims,N]);
        for o = 1:length(obstacles)
            for a = 1:N
                dist2obsts(a) = norm(agent_cbf_pos(:,a) - obstacles(o).center') - obstacles(o).radius;
                grad_dist2obst(a,:) = (agent_cbf_pos(:,a) - obstacles(o).center') / dist2obsts(o); % gradj
            end
        end
        grad_h = blkdiag(grad_dist2obst(1,:));
        for a = 2:N
            grad_h = blkdiag(grad_h,grad_dist2obst(a,:));
        end
            
        A = - grad_h;
        b = cbf_alpha * dist2obsts;

        % Store the distance of the agents to objects
        Agent2Obst_dists = [Agent2Obst_dists, dist2obsts'];
        grad_Agent2Obst = cat(3,grad_Agent2Obst,grad_dist2obst);
        grads_hs = cat(3,grads_hs,grad_h);
        opt_hs = [opt_hs, dist2obsts'];

    else
        % Compute the MaxStress
        MaxStress = yield_stress/SF;
        H = 2 * eye(3*N);
        f = -2 * u(:);
    
    
        if it_loop > niters/2 && forceCBF
            % A*u >= b --> La cbf actúa
            A = ones(N,3*N);
            b = - cbf_alpha * ones(N,1) * MaxStress;

            % Store the h gradients
            grads_hs = cat(3,grads_hs,ones(N,3*N));
        else
            % A*u <= b --> La cbf no actúa
            A = - eye(N,3*N);
            b = cbf_alpha * ones(N,1) * MaxStress;

            % Store the h gradients
            grads_hs = cat(3,grads_hs,eye(N,3*N));
        end

        % store the h values
        opt_hs = [opt_hs, (ones(N,1) * MaxStress)'];
    end

    % Set optimization options
    options = optimset('Display', 'off');

    % Solve QP for velocity
    u_cbf = quadprog(H, f, A, b, [], [], [], [], [], options);
    % [u_cbf, fval, exitflag] = quadprog(H, f, A, b);
    % u_cbf = kCBF * u_cbf;

    As = [As, A*u_cbf];
    bs = [bs, b'];


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
        if obstacle_avoidance
            u_obst = u_obst(:);
            prop_nv_cbf(i,1) = norm(u_obst(3*i-2:3*i,1));
        end
    end
    
    % Store current velocity data
    vs = [vs u];
    nvs = [nvs nv];

    vs_cbf = [vs_cbf u_cbf];
    nvs_cbf = [nvs_cbf nv_cbf];

    if obstacle_avoidance
        prop_vs_cbf = [prop_vs_cbf u_obst];
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

    %% Pause for live plot
    if plotLiveCBF
        pause(0.005) % we give some time for the user to visualize the created plot
    end

end

%% Create plotting and video
% Viewing angles for the 3D plot

if waypoints
    view_1 = 145.4729;
    view_2 = 29.0838;
else
    view_1 = 14.1759;
    view_2 = 18.4354;
end
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
    grid on;
    legend show;
end

%% Plot Distance to object
if obstacle_avoidance
    figure;
    hold on;
    for a = 1:N
        plot(Agent2Obst_dists(a,:),'Color', color_robots(a),'DisplayName',strcat('Agent ',num2str(a)));
    end
    hold off;
    title("Distance to objects");
    xlabel("Iters");
    ylabel("Distance (cm)");
    grid on;
    legend show;
end

%% Plot gradient of the distance to object
if obstacle_avoidance
    figure;
    % X
    subplot(3,1,1);
    hold on;
    for a = 1:N
        plot(squeeze(grad_Agent2Obst(a,1,:)),'Color', color_robots(a),'DisplayName',strcat('Agent ',num2str(a)));
    end
    hold off;
    title("Gradient Distance to Object - X");
    xlabel("Iters");
    ylabel("Distance (cm)");
    grid on;
    legend show;

    % Y
    subplot(3,1,2);
    hold on;
    for a = 1:N
        plot(squeeze(grad_Agent2Obst(a,2,:)),'Color', color_robots(a),'DisplayName',strcat('Agent ',num2str(a)));
    end
    hold off;
    title("Gradient Distance to Object - Y");
    xlabel("Iters");
    ylabel("Distance (cm)");
    grid on;
    legend show;

    % Z
    subplot(3,1,3);
    hold on;
    for a = 1:N
        plot(squeeze(grad_Agent2Obst(a,3,:)),'Color', color_robots(a),'DisplayName',strcat('Agent ',num2str(a)));
    end
    hold off;
    title("Gradient Distance to Object - Z");
    xlabel("Iters");
    ylabel("Distance (cm)");
    grid on;
    legend show;

end

%% Gradient Norm

if obstacle_avoidance
    figure;
    hold on;
    for a = 1:N
        for i = 1:niters
            norm_grads(a,i) = norm(grad_Agent2Obst(a,:,i));
        end
        plot(norm_grads(a,:),'Color', color_robots(a),'DisplayName',strcat('Agent ',num2str(a)));
    end
    % ylim([0 3])
    hold off;
    title("Norm grad distance");
    xlabel("Iters");
    ylabel("Distance (cm)");
    grid on;
    legend show;
end

%% Individual Velocity Correction plot
if plotCBFVelCorr
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

if plotCBFVelDiff
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

if plotVelComp

    figure
    plot(nvs(1,:)');
    hold on;
    plot(nvs_cbf(1,:)');
    if obstacle_avoidance
        plot(prop_vs_cbf(1,:)');
    end
    hold off;
    title("Velocity Norm Comparison");
    xlabel("Iters");
    ylabel("m/s");
    % legend(["Agend 1 - std", "Agend 2 - std", "Agend 3 - std", "Agend 4 - std", "Agend 1 - cbf", "Agend 2 - cbf", "Agend 3 - cbf", "Agend 4 - cbf"]);
    if obstacle_avoidance
        legend(["Agend 1 - std", "Agend 1 - cbf", "Agent 1 - Proposed cbf"]);
    else
        legend(["Agend 1 - std", "Agend 1 - cbf"]);
    end

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