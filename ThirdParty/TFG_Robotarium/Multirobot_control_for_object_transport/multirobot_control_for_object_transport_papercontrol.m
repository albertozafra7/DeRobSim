% Multirobot control in a 2D workspace with a team of double-integrator mobile robots
% for deformable object transport

% A team of robots moves from an initial to a desired configuration in the workspace 
% The desired configuration is defined by four parameters: shape, size, centroid, and rotation 
% The idea of the controller is to maintain, to the extent possible, the desired shape along the way
% This shape maintenance behavior is interesting for, e.g., transporting an object 
% This script below simulates the motion of the robots
% The behavior of the deformable object is simulated using the
% As-Rigid-As-Possible deformation model, programmed in the auxiliary files provided with this script

% The code is based on the paper "Multirobot control with double-integrator dynamics and
% control barrier functions for deformable object transport," by 
% R. Herguedas, M. Aranda, G. Lopez-Nicolas, C. Sagues and Y. Mezouar, IEEE ICRA 2022
% The pdf of the paper is included in the auxiliary files folder

% Author: Raquel Marcos, September 2022
% Initial version by: Miguel Aranda, August 2022

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars 
close all

% We add the paths of the folder and subfolders we will need.

Robotarium_path = genpath('Robotarium_simulator');
if(isempty(Robotarium_path))
    disp('WARNING: Cannot find utilities directory.  This script should be run from the base directory.')
else
    addpath(Robotarium_path)
end

Aux_path = genpath('multirobot_control_transport_aux_files');
if(isempty(Aux_path))
    disp('WARNING: Cannot find utilities directory.  This script should be run from the base directory.')
else
    addpath(Aux_path)
end


%% Choice of general parameters of the simulation

with_object = 0;        % 0: simulation without object, 1: with object
with_obstacles = 0;     % 0: without obstacles; 1: with obstacles
show_rest_shape = 0;    % 1: plots the object's rest shape and node indexes (we can use this plot to define the nodes where the robots are placed)
show_figures = 0;       % 0: does not show figures during execution, 1: shows them
save_plots = 0;         % 0: does not save plots; 1: save plots
make_video = 0;         % 0: does not make a video, 1: makes it
ob_type = 2;            % select type of object: 1 for sheet, 2 for linear (rod, cable)
create_folder = 0;      % 0: does not create a folder; 1: creates a folder

if create_folder == 1
    f_name = 'object_1_si_papercontrol_simulation';   % folder name for storing the results 
    folder_result = strcat('./results_',f_name,'/');    % it's a path
    if exist(char(folder_result),'dir') == 0            % 'dir' - Just folders
       mkdir(char(folder_result));                      % create the folder
    end
end


%% We define the object's nodes arranged in a rectangular grid

if ob_type == 1 % SHEET
    nx = 8;     % number of nodes in x direction of the grid
    ny = 8;     % number of nodes in y direction of the grid
else            % LINEAR
    nx = 80; 
    ny = 1;     % only 1 because the object's shape is a straight line
end

sx = 2;     % length of the object in x direction (in the graphics, it will be the 'y' direction)
sy = 2.5;   % length of the object in y direction (only relevant if ob_type = 1) (in the graphics, it will be the x direction)

% We need to reduce the defined object for the Robotarium experiment because its
% working area is smaller.
if ob_type == 1
    scale_ob = 0.3;
else
    scale_ob = 0.8;
end

omesh = create_mesh(nx, ny, sx * scale_ob, sy * scale_ob);    % mesh of the object: its shape and the links between nodes
Pob0 = omesh.shape';                                          % "rest" shape of the object (x,y,z coordinates for each node)


%% We plot the mesh of the object in its rest state and the node indexes

if show_rest_shape == 1
    figure(1)
    plot_shape(Pob0, [omesh.elements(:,[1 2]); omesh.elements(:,[1 3]); omesh.elements(:,[2 3])], [], 'b')
    for i = 1:nx*ny
        text(Pob0(1,i), Pob0(2,i), Pob0(3,i), int2str(i))
    end
    title('Rest shape of the object with the indexes of its nodes')
    sxym = max(sx,sy);
    xlim([-0.6*sxym 0.6*sxym])
    ylim([-0.6*sxym 0.6*sxym])
    zlim([-0.5 0.5])
    view(-50, 60)       % Set the angle of the view from which an observer sees the current 3-D plot
    disp('Press a key to continue the execution')   % For selecting the nodes of the object where we place the robots
    pause
    disp('---')
    disp('---')
    disp('The simulation is running...')
end


%% We initialize the model (As-Rigid-As-Possible) of the deformable object
%  And we also define the desired configuration of the team of robots, as they are
%  attached to the object

if ob_type == 1 % SHEET

    % The sheet is simulated in 3D, but we will only display its projection
    % on the XY plane

    handled_nodes = [1 8 64 57];                     % the nodes of the object where we place the robots (we can select them using the previous figure)
    N = length(handled_nodes);                       % number of robots (equal to the number of object nodes they are attached to)
    g0 = [0.8; -0.5];                                % centroid of the rest position (Robotarium coordinates are different from the paper ones)
    P0 = Pob0(1:2, handled_nodes) + g0 * ones(1,N);  % positions of the robots (2D) when the object is at rest (projection on the xy plane)
    th0 = pi*ones(1,N);                              % radians
    n_iters_arap = 3;                                % choose a value of 2 or 3: the higher the value the stiffer the simulated object
    arap_params = create_params_for_arap(omesh, handled_nodes, [], n_iters_arap);  % create model parameters
    
    % We define the target shape for the team of robots, Pd
    gd = [-0.8; 0.25];      % desired centroid
    Pd0 = P0;               % the target shape is the same as the initial one (but we will scale it, rotate it, translate it)
    s0 = 1.2;               % for convenience we define an initial scaling (not used in the paper)
    a0 = -55 * pi/180;      % for convenience we define an initial rotation (not used in the paper)
    R0 = [cos(a0) -sin(a0); sin(a0) cos(a0)];   % Rotation matrix
    Pd0 = s0 * R0 * Pd0;    % from Pd0 we will define the target shape Pd (see below)

else % LINEAR OBJECT

    % The linear object is simulated in 2D 
    
    handled_nodes = [1 20 40 60 80];                % the nodes of the object where we place the robots (we can select them using the previous figure)
    N = length(handled_nodes);                      % number of robots (equal to the number of object nodes they are attached to)
    g0 = [-1.3; 0];                                 % centroid of the rest position (Robotarium coordinates are different from the paper ones)
    P0 = Pob0(1:2, handled_nodes) + g0 * ones(1,N); % positions of the robots (2D) when the object is at rest
    th0 = zeros(1,N);                               % radians
    omesh.shape = omesh.shape(:, 1:2);              % it's in 2D, not 3D
    n_iters_arap = 3;                               % choose a value of around 3 for less stiff (cable), around 10 or more for stiffer (elastic rod)
    arap_params = create_params_for_arap_2d(omesh, handled_nodes, [], n_iters_arap);    % create model parameters

    % We define the target shape for the team of robots, Pd
    gd = [0.75; 0.25];                          % desired centroid
    Pd0 = [0 0; -1 0; -1 1; -1 2; 0 2]';
    s0 = 0.4;                                   % for convenience we define an initial scaling (not used in the paper)
    a0 = 25 * pi/180;                           % for convenience we define an initial rotation (not used in the paper)
    R0 = [cos(a0) -sin(a0); sin(a0) cos(a0)];   % Rotation matrix
    Pd0 = s0 * R0 * Pd0;                        % from Pd0 we will define the target shape Pd (see below)

end

% We can now define Pd, whose centroid is gd (for convenience)
Pd = (Pd0 - (1/N) * Pd0 * ones(N,1) * ones(1,N)) + gd * ones(1,N);  % Pd is Pd0 moved to the center and then to the desired position

% % Plot target shape
% figure(2)
% plot(Pd(1,:), Pd(2,:), 'o', 'linewidth', 3, 'markersize', 8)
% hold on
% plot(gd(1), gd(2), 'rx', 'linewidth', 3, 'markersize', 8)
% title('Target shape Pd')
% axis equal

% Target formation scale and rotation. Since we have already rotated and scaled
% Pd when we defined it above, then we can select sd=1, thd=0 (as done in the experiments presented in the ICRA 22 paper)
sd = 1;
thd = 0; % in radians
Rd = [cos(thd) -sin(thd); sin(thd) cos(thd)];

% Target configuration (eq. 2 of the paper)
PT = sd * Rd * (Pd - gd * ones(1,N)) + gd * ones(1,N);


%% We define the initial conditions of the simulation and we define the required simulation parameters

% Initial configuration of the robots
P = P0;   % we choose it as the positions of the robots when the object is in its rest shape
P_teor = P0;

% Initial velocity of the robots
v = 0;

% % Plot target and initial configuration
% figure(3)
% plot(PT(1,:), PT(2,:), 'o', 'linewidth', 3, 'markersize', 8)
% title('Target configuration PT')
% axis equal

% Initial configuration of the object
Pob = Pob0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Each of the following will be a growing array that will store the values of variables at each iteration
gammas = [];
egs = [];
ess = [];
eths = [];
Hs = [];
Ps = [];
P_sis = [];
P_teors = [];
Pbs = [];
Pdots = [];
Pbdots = [];
Hdots = [];
Ufs = [];
as = [];
vs = [];
dx_unis = [];
nvs = [];
nas = [];

dt = 0.033;     % Time step of the simulation
nit = 1500;     % Number of iterations of the control loop. The simulated time in seconds will be nit*dt

% For plotting: we define the approx. limits of the space that will be covered by the
% robots during the simulation
xmin = min([P0(1,:), PT(1,:)]);
xmax = max([P0(1,:), PT(1,:)]);
ymin = min([P0(2,:), PT(2,:)]);
ymax = max([P0(2,:), PT(2,:)]);
plotaxfont = 'Helvetica';
ind_im = 1;     % Index of images (used for creation of video)
fps = 10;       % Frames per second for the video

% Create figure from which we will create the video frames
fi = figure(20);
if show_figures == 0
    set(fi, 'visible', 'off')
end

tic
%% Robotarium Initialization
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', [P0; th0]);
projection_distance = 0.05;
if ob_type == 1
    margin = 0.02;
else
    margin = 0.02;
end
ang_vel_limit = pi/10;

% We initialize controllers and barrier function to avoid collisions
multirobot_controller = create_multirobot_controller('sd', sd, 'dt', dt, 'asat', 1e10, 'k1H', 0.1, 'k2H', 0.2, 'k1G', 0.1, 'k2G', 0.2, ...
    'k1s', 0.1, 'k2s', 0.2, 'k1gm', 0.05, 'k2gm', 0.4, 'k1th', 0.2, 'k2th', 0.1);
% multirobot_controller = create_multirobot_controller('sd', sd, 'dt', dt, 'asat', 1e10, 'k1H', 0.1, 'k2H', 0, 'k1G', 0.1, 'k2G', 0, ...
%     'k1s', 0.1, 'k2s', 0, 'k1gm', 0.05, 'k2gm', 0, 'k1th', 0.2, 'k2th', 0);
% multirobot_controller = create_multirobot_controller('sd', sd, 'dt', dt, 'asat', 1e10, 'k1H', 0.5, 'k2H', 0, 'k1G', 0.1, 'k2G', 0, ...
%      'k1s', 0.2, 'k2s', 0, 'k1gm', 0.35, 'k2gm', 0, 'k1th', 0.2, 'k2th', 0);
[si_to_uni_dynamics, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', projection_distance);
barrier_certificate = create_uni_barrier_certificate_with_boundary();

% We save the current initial positions of the robots
robot_states = r.get_poses();
r.step();

% We define the positions of the obstacles
if with_obstacles == 1
    if ob_type == 1
        obst_1 = [-0.4; -0.6];
        obst_2 = [-0.3; -0.7];
        obst_3 = [-0.2; -0.8];
        obst_4 = [-0.1; -0.9];
        obst_5 = [0; -1];
        obst_6 = [1.6; 0];
        obst_7 = [0.3; 0];
        obst_8 = [0.2; 0];
        obst_9 = [0.1; 0.1];
        obst_10 = [0; 0.2];
        obst_11 = [-0.1; 0.3];
        obst_12 = [-0.2; 0.4];
        obst_13 = [-0.3; 0.5];
        obst_14 = [-0.4; 0.6];
        obst_15 = [-0.5; 0.7];
        obst_16 = [-0.55; 0.8];
        obst_17 = [-0.55; 0.9];
        obst_18 = [-0.55; 1];
        
        obstacles = [obst_1 obst_2 obst_3 obst_4 obst_5 obst_6 obst_7 obst_8 obst_9 obst_10...
                     obst_11 obst_12 obst_13 obst_14 obst_15 obst_16 obst_17 obst_18];
        plot(obstacles(1,1:5), obstacles(2,1:5), 'k-', 'LineWidth', 2);
        plot(obstacles(1,6:end), obstacles(2,6:end), 'k-', 'LineWidth', 2);
    
    else
    
        obst_1 = [-0.3; -0.7];
        obst_2 = [-0.3; -0.8];
        obst_3 = [-0.3; -0.9];
        obst_4 = [-0.3; -1];
        obst_5 = [-0.2; 0.4];
        obst_6 = [-0.3; 0.45];
        obst_7 = [-0.4; 0.5];
        obst_8 = [-0.5; 0.55];
        obst_9 = [-0.6; 0.6];
        obst_10 = [-0.7; 0.65];
        obst_11 = [-0.85; 0.9];
        obst_12 = [-0.85; 1];
    
        obstacles = [obst_1 obst_2 obst_3 obst_4 obst_5 obst_6 obst_7 obst_8 obst_9 obst_10 ...
                     obst_11 obst_12];
        plot(obstacles(1,1:4), obstacles(2,1:4), 'k-', 'LineWidth', 2);
        plot(obstacles(1,5:end), obstacles(2,5:end), 'k-', 'LineWidth', 2);
    end
else
    obstacles = [];
end

%% Plotting

% Initial and desired final positions
colour = ["#D80909", "#414FD7", "#179115", "#EE0DC8", "#CF7F04"];
for i = 1:N
    uistack(plot(P0(1,i), P0(2,i), '.', 'Color', colour(i), 'MarkerSize', 14), 'bottom');
    plot(PT(1,i), PT(2,i), 'o', 'Color', colour(i), 'MarkerSize', 7, 'LineWidth', 1.5);
end

% We initialize the object plotting
if with_object
    if ob_type == 1
        
        Pob = simulate_object_arap(arap_params, Pob, [robot_states(1:2,:); zeros(1,N)]);
        
        for i = 1:size(arap_params.edg, 1)   % We plot the edges of the sheet's mesh between nodes               
        object_mesh(i) = plot([Pob(1,arap_params.edg(i,1)) Pob(1,arap_params.edg(i,2))], ...
                         [Pob(2,arap_params.edg(i,1)) Pob(2,arap_params.edg(i,2))], ...
                         '-', 'color', [0.5 0.5 0.8], 'linewidth', 1);
        end
    
    else            
        
        Pob(1:2,:) = simulate_object_arap_2d(arap_params, Pob(1:2, :), robot_states(1:2,:));
        object_mesh = plot(Pob(1,:), Pob(2,:), '-', 'color', [0.5 0.5 0.8], 'linewidth', 2);   % For the linear object, we just plot a chain of nodes
    
    end
end


%% Control loop

for it_loop = 1:nit

    robot_states = r.get_poses();   % We save the current states (position + orientation) of the robots
    
    g = (1/N) * robot_states(1:2,:) * ones(N,1);  % Current formation centroid
    Pb = robot_states(1:2,:) - g * ones(1,N);     % eq. 4
    Pdb = PT - gd * ones(1,N);  % eq. 5
    Pdbp = pinv(Pdb);           % Pseudoinverse
    S = [0 -1; 1 0];
    h1 = trace(Pb * Pdb') / trace(Pdb * Pdb');          % eq. 7
    h2 = trace(Pb * (S * Pdb)') / trace(Pdb * Pdb');
    H = [h1 -h2; h2 h1];
    gamma = (1/2) * norm(Pb - H * Pdb, 'fro').^2;       % eq. 3
    s = sqrt(det(H));           % Current scale of the formation
    th = atan2(h2, h1);         % Current rotation of the formation
    eg = g - gd;                % Position error
    es = s - sd;                % Scale error
    eth = th - thd;             % Rotation error

    % We plot the object shape in each iteration
    if with_object
        if ob_type == 1
            
            Pob = simulate_object_arap(arap_params, Pob, [robot_states(1:2,:); zeros(1,N)]);
            
            for i = 1:size(arap_params.edg, 1)
                object_mesh(i).XData = [Pob(1,arap_params.edg(i,1)) Pob(1,arap_params.edg(i,2))];
                object_mesh(i).YData = [Pob(2,arap_params.edg(i,1)) Pob(2,arap_params.edg(i,2))];
            end
    
        else            
            
            Pob(1:2,:) = simulate_object_arap_2d(arap_params, Pob(1:2, :), robot_states(1:2,:));
            object_mesh.XData = Pob(1,:);
            object_mesh.YData = Pob(2,:);
    
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    % MULTIROBOT CONTROL

    si_states = uni_to_si_states(robot_states);            % Projects the single-integrator system a distance in front of the unicycle system

    % To correct the final error of the position
    projected_PT = zeros(2,N);

    for i = 1:N

        projected_PT(1, i) = PT(1, i) + (projection_distance * cos(robot_states(3,i)));
        projected_PT(2, i) = PT(2, i) + (projection_distance * sin(robot_states(3,i)));

    end

    [a, data] = multirobot_controller(si_states, projected_PT);      % The controller returns the acceleration and useful data
    disp(a);

    v = v + a * dt;     % Velocity update of the robots
%     v = a;              % If we consider a single-integrator controller
    
    % We can saturate the velocities, i.e., we only allow a maximum velocity (in norm) vsat.
    % This makes the motion more natural, avoiding very high velocities at the first time
    % instants, which are not achievable by real mobile robots.
    % NOTE: Saturation was not used in the paper. We can omit the saturation if the controller
    % works well without it. To omit the saturation, simply choose very high values of vsat (1e10).
    
    vsat = 0.15;
    for i = 1:N
        if norm(v(:,i)) > vsat
            v(:,i) = vsat * v(:,i) / norm(v(:,i));
        end
    end
        
    % To stop each robot when it is near its final position
    for i = 1:N
        
        si_to_projected_vec = projected_PT(1:2, i) - si_states(1:2, i);
        
        if (norm(si_to_projected_vec) <= margin)
            v(:,i) = zeros(2,1);
        end
    
    end
        
%     % To stop the complete formation when its centroid is close to the
%     % desired final centroid
%     si_to_projected_vec = g - gd;
%     
%     if (norm(si_to_projected_vec) <= margin)
%         v = zeros(2,N);
%     end

%     % To stop the complete formation when every robot is close to its final
%     % position 
%     si_to_projected_vec = projected_PT(1:2, :) - si_states(1:2, :);
%     correct_position = zeros(1,N);
% 
%     for i=1:N
%         if (norm(si_to_projected_vec(1:2,i)) <= margin)
%             correct_position(i) = 1;
%         end
%     end
%     
%     if (correct_position == ones(1,N))
%         v = zeros(2,N);
%     end

    
    for i = 1:N 
        nv(i) = norm(v(:,i));     % We store the norms of velocity
    end

    dx_uni = si_to_uni_dynamics(v, robot_states);       % We adapt the velocity to one suitable for unycycle dinamics
    dx_uni(2,:) = ang_vel_limit * dx_uni(2,:) / (pi/2);

    dx_uni = barrier_certificate(dx_uni, robot_states, obstacles); % We apply the barrier function to avoid collisions

    r.set_velocities(1:N, dx_uni);
    r.step();

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % We store the values at the current iteration
    gammas = [gammas; gamma];   % We store the value of gamma at this iteration
    Hs = [Hs; H];
    Ps = [Ps; robot_states(1:2,:)];
    P_sis = [P_sis; data.P];
    Pbs = [Pbs; Pb];
    egs = [egs; norm(eg)];  
    ess = [ess; es];
    eths = [eths; eth];      
    as = [as; data.a];
    vs = [vs; v];
    dx_unis = [dx_unis; dx_uni];
    nvs = [nvs; nv];
    nas = [nas; data.na];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    if save_plots  % we simulate the behavior of the real object
    
        % We create the frame for the video, including object and robots
        if it_loop==1 || rem(it_loop, floor(1/(fps*dt))) < 1e-8    % we do this because checking "rem == 0" can be unreliable

            % We can save the figure and print it into image files (eps, png)
            hgsave(strcat(folder_result,'/','robots_object'));
            print(strcat(folder_result,'/','robots_object'),'-depsc')
            print(strcat(folder_result, '/', 'robots_object_', int2str(ind_im)), '-dpng', '-r300');
            ind_im = ind_im + 1;    % increase the index for the next frame
            if show_figures == 1    % we give some time for the user to visualize the created plot
                pause(0.1)
            end

            % % The following commented lines are for plotting the object in 3D
            % if ob_type==1
            % plot_shape(Pob,[omesh.elements(:,[1 2]);omesh.elements(:,[1 3]);omesh.elements(:,[2 3])],[],'b')
            % for i=1:N
            %     plot3(PT(1,i),PT(2,i),0,'ro','markersize',12,'linewidth',3,'markerfacecolor','w') 
            %     plot3(P(1,i),P(2,i),0,'bo','markersize',8,'linewidth',3,'markerfacecolor','b') 
            % 
            % end
            % zlim([-2 2])
            % view(-30,30)
            
        end
    end
end

% We save all variables of the workspace
% save(fullfile(strcat(folder_result, '/', 'Object_1_si_papercontrol_simulation.mat')))
% save('Object_1_obstacles_robotarium.mat')


% Plots
% plots_and_video_papercontrol

r.debug();
toc