use_waypoints = false;
use_rotation_z = false;
use_obstacles = false;
moveNdim = 3;
force_CBF = false;
use_real_CBF = true;
N = 8;
dt = 0.01;
niters = 1700;
E = 3000;
nu = 0.01;
kext = 10;
yield_stress = 210e2;
SF = 2.5;
u_sat = 1.5;
k_H = 1;        % for moving toward shape-preserving transformation
k_G = 50;         % for moving toward a configuration consistent with our deformation modes
k_c = 1;         % for centroid translation
k_s = 0.5;        % for scaling
k_Hd = 0.8;      % for scaling and orientation
kCBF = 1;        % for controllig the stress avoidance influence
cbf_alpha = 10;   % for controlling the risks taken by the barrier function
sd = 3.5;
thd = 0;
p_init = [ 0.5,  0.5,  1.0;...
           0.5, -0.5,  1.0;...
          -0.5, -0.5,  1.0;...
          -0.5,  0.5,  1.0;...
           0.5,  0.5,  0.0;...
           0.5, -0.5,  0.0;...
          -0.5, -0.5,  0.0;...
          -0.5,  0.5,  0.0];
agent_destinations = [ 0.5,  0.5,  1.0;... % Agent 1
                           0.5, -0.5,  1.0;... % Agent 2
                          -0.5, -0.5,  1.0;... % Agent 3
                          -0.5,  0.5,  1.0;... % Agent 4
                           0.5,  0.5,  0.0;... % Agent 5
                           0.5, -0.5,  0.0;... % Agent 6
                          -0.5, -0.5,  0.0;... % Agent 7
                          -0.5,  0.5,  0.0]'...% Agent 8
                          ;
recordVideo = false;
videoPrefix = 'simulation_Exp1';


results = runCubeKabschExperiment('use_waypoints',use_waypoints,'use_rotation_z', use_rotation_z,...
    'use_obstacles', use_obstacles,'moveNdim',moveNdim,'force_CBF',force_CBF,'use_real_CBF',use_real_CBF,...
'N',N,'dt',dt,'niters',niters,'E',E, 'nu', nu,'kext', kext,'yield_stress', yield_stress,'SF', SF,'u_sat', u_sat,...
'k_H',k_H,'k_G',k_G,'k_c',k_c,'k_s', k_s,'k_Hd', k_Hd,'kCBF', kCBF,'cbf_alpha', cbf_alpha,'sd', sd,'thd', thd,...
'p_init', p_init,'agent_destinations',agent_destinations,'recordVideo',recordVideo,'videoPrefix',videoPrefix);