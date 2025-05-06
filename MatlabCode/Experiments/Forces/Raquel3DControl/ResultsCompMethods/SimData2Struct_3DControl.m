% Save all the data in a struct for plotting it later or saving it in a
% .mat file

% Simulation params
data_standard.niters = niters; % Number of iterations
data_standard.dt = dt; % timestep

% Obstacles
if exist('obstacles1', 'var')
    data_standard.obstacles1 = obstacles1;
end

if exist('obstacles2', 'var')
    data_standard.obstacles2 = obstacles2;
end

if exist('obstacles', 'var')
    data_standard.obstacles = obstacles;
end

% Positions
data_standard.p0 = p0; % Initial positions of the agents (3*N_agents x 1)
data_standard.g0 = g0; % Initial centroid position (3 x 1)
data_standard.PT = PT; % Final positions of the agents (3*N_agents x 1)
data_standard.gd = gd; % Final centroid position (3 x 1)
data_standard.ps = ps_3D; % Positions of the agents in each iteration (3*N_agents x N_iters)

% Velocities
data_standard.vs = vs_3D; % Velocities of the agents in each iteration (3*N_agents x N_iters)
data_standard.nvs = nvs_3D; % Velocity norms of the agents in each iteration (N_agents x N_iters)

% Errors
data_standard.gammas_H = gammas_H_3D; % Cost relative to shape-preserving transformation (N_iters x 1)
data_standard.gammas_G = gammas_G_3D; % Cost relative to configuration consistent with our deformation modes (N_iters x 1)
data_standard.egs = egs_3D; % Centroid position errors (N_iters x 1)
data_standard.ess = ess_3D; % Scale errors (N_iters x 1)
data_standard.eths = eths_3D; % Rotation errors (N_iters x 1)
data_standard.eth_xs = eth_xs_3D; % Roll rotation error (N_iters x 1)
data_standard.eth_ys = eth_ys_3D; % Pitch rotation error (N_iters x 1)
data_standard.eth_zs = eth_zs_3D; % Yaw rotation error (N_iters x 1)

% Plotting
data_standard.pairs = pairs; % Interconnection between agents
data_standard.plotTitleSufix = ""; % Plotting title sufix (String)
data_standard.x_limit = x_limit; % Plotting limits (2 x 1) float vector
data_standard.y_limit = y_limit; % Plotting limits (2 x 1) float vector
data_standard.z_limit = z_limit; % Plotting limits (2 x 1) float vector
data_standard.plotaxfont = plotaxfont; % Font of the plotting (String)
data_standard.color_graphs = color_graphs; % Colors of the errors (N_errors x 1) (Strings)
data_standard.color_robots = color_robots; % Colors of the agents (N_agents x 1) (Strings)
data_standard.view_1 = view_1; % Viewing angle in degs (Float)
data_standard.view_2 = view_2; % Viewing angle in degs (Float)
data_standard.save_plots = save_plots; % If we want to save the plots (Bool)

if exist('folder_result', 'var')
    data_standard.folder_result = folder_result; % Path to the parent folder where we want to save the plots (String)
end