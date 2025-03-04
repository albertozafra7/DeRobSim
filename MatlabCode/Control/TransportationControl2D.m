% Control of a team of mobile robots in a 3D working area with double-integrator dynamics.

% The goal of the controller is to keep, to the extent possible, the shape
% of the robots formation on its way from an initial to a desired configuration
% with different shape, size, centroid and rotation.

% The controller itself calculates the acceleration, from which the required velocity
% for the robots to reach the desired position in each iteration is obtained.
% positions - Current positions of the robots
% destinations - Desired final positions of the robots
% velocities - Required velocities
% data - Object in which other necessary variables are storaged

% The code is based on the paper "Multirobot control with double-integrator dynamics and
% control barrier functions for deformable object transport," by 
% R. Herguedas, M. Aranda, G. Lopez-Nicolas, C. Sagues and Y. Mezouar, IEEE ICRA 2022

% Author: Miguel Aranda, August 2022
% Co-Author: Raquel Marcos, September 2022
% Modification: Alberto Zafra-Navarro, August 2024

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ a ] = TransportationControl2D(positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat)

    % ++++++++++ Optional parameters evaluation ++++++++++

    % Deformation control gains
    if ~exist('k1H','var')
        k1H = 6;
    end
    if ~exist('k2H','var')
        k2H = 3;
    end

    % Deformation Correction control gains
    if ~exist('k1G','var')
        k1G = 3;
    end
    if ~exist('k2G','var')
        k2G = 2;
    end

    % Scale control gains
    if ~exist('k1s','var')
        k1s = 5;
    end
    if ~exist('k2s','var')
        k2s = 6;
    end

    % Position control gains
    if ~exist('k1g','var')
        k1g = 2;
    end
    if ~exist('k2g','var')
        k2g = 6;
    end

    % Orientation control gains
    if ~exist('k1th','var')
        k1th = 4;
    end
    if ~exist('k2th','var')
        k2th = 4;
    end

    % Deformation control weights
    if ~exist('alpha_H','var')
        alpha_H = 4; % Deformation term weight
    end
    if ~exist('alpha_G','var')
        alpha_G = 2; % Correction term weight
    end

    % Time step
    if ~exist('dt','var')
        dt = 0.033;
    end

    % Target formation scale
    if ~exist('sd','var')
        sd = 1;
    end

    % Target formation rotation
    if ~exist('thd','var')
        thd = 0;
    end

    % Max robot acceleration (in norm) allowed
    if ~exist('asat','var')
        asat = 1e10;
    end

    % ++++++++++++++++++++++++++++++++++++++++++++++++++++

    % ++++++++++ Properties Configuration ++++++++++
    S = [0 -1; 1 0];    % 90 deg rotation matrix to be used in the control

    % Number of positions and destinations
    N_pos = size(positions, 2);
    N_des = size(destinations, 2);
    assert(N_pos == N_des, 'Column size of current position (%i) and final position (%i) must be equal', N_pos, N_des);

    % -- Matricial forms --
    N = N_pos;          % Number of robots
    P = positions;
    Pd = destinations;

    % -- Desired --
    gd = compute_centroid(Pd);
    Pdb = compute_relativePos(Pd,gd); % eq. 5
    Pdbp = pinv(Pdb); % Pseudoinverse

    % -- Current --
    g = compute_centroid(P);  % Current formation centroid
    Pb = compute_relativePos(P,g);     % eq. 4
    [H, h1, h2] = compute_H(Pb,Pdb,S);

    % -- Previous (Used for computing the derivatives) --
    prev_P = prev_pos;
    prev_g = compute_centroid(prev_P);
    prev_Pb = compute_relativePos(prev_P,prev_g);
    [prev_H, ~, ~] = compute_H(Pb,Pdb,S);
    % h1 = H(1,1);
    % h2 = -H(1,2);
    
    % -- Errors --
    gamma = (1/2) * norm(Pb - H * Pdb, 'fro').^2;       % eq. 3 % Deformation difference
    s = sqrt(det(H));           % Current scale of the formation
    th = atan2(h2, h1);         % Current rotation of the formation
    eg = g - gd;                % Position error
    es = s - sd;                % Scale error
    eth = th - thd;             % Rotation error

    % -- Derivatives --

    % We compute the time variations (Pdot, Pbdot, Hdot) we will need for the controllers.
    % At the first time instant, the robots are static so all time variations are zero
    if prev_P == 0      % There's no previous iteration; i.e. this is the first one
        Pdot = zeros(2,N);
        Pbdot = zeros(2,N);
        Hdot = zeros(2,2);

    else

        Pdot = (P - prev_P) / dt;       % Change of P relative to the previous iteration
        Pbdot = (Pb - prev_Pb) / dt;    % Change of Pb relative to the previous iteration
        Hdot = (H - prev_H) / dt;       % Change of H relative to the previous iteration

    end

    % ++++++++++++++++++++++++++++++++++++++++++++++++++++

    % ++++++++++ Control Algorithm ++++++++++

    % We compute the different controllers (i.e., the additive terms that make up
    % the full control law) using the variables we have computed above:

    U_H = k1H * (H * Pdb - Pb) + k2H * (Hdot * Pdb - Pbdot);                    % eq. 8

    % As Matlab has a lot of precision we decided to round up the results
    % for ensuring that a value close to 0 is 0
    k1G_multiplier = (Pb * Pdbp * Pdb - Pb);
    k1G_multiplier(k1G_multiplier < 10e-5) = 0;

    U_G = k1G * k1G_multiplier + k2G * (Pbdot * Pdbp * Pdb - Pbdot);            % eq. 9
    
    U_gamma = alpha_H * U_H + alpha_G * U_G;                                    % eq. 10

    U_s = -k1s * es * (1/s) * H * Pdb - k2s * Pbdot;                            % eq. 17

    U_D = U_gamma + U_s;                                                        % eq. 18

    U_g = -k1g * eg * ones(1,N) - k2g * Pdot;                                   % eq. 19

    U_th = -k1th * eth * S * H * Pdb - k2th * Pbdot;                            % eq. 20
    
    U_f = U_D + U_g + U_th;                                                     % full control, eq. 22
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % We move the robots.        
    a = U_f;    % Acceleration to be applied for the computed full control (x,y) x N_ROBOTS

    % We can saturate the accelerations, i.e., we only allow a maximum acceleration of value asat (in norm).
    % This makes the motion more natural, avoiding very high accelerations at the first time instants, which
    % are not achievable by real mobile robots.
    % NOTE: Saturation was not used in the paper. We can omit the saturation if the controller works well without it. 
    % To omit the saturation, simply choose very high values of asat (1e10).

    for i = 1:N
        if norm(a(:,i)) > asat
            a(:,i) = asat * a(:,i) / norm(a(:,i));
        end
    end

end

function [g] = compute_centroid(P)
    N = size(P, 2);
    g = (1/N) * P * ones(N,1);
end

function [Pb] = compute_relativePos(P,g)
    N = size(P, 2);
    Pb = P - g * ones(1,N); % eq. 5
end

function [H, h1, h2] = compute_H(Pb,Pdb, S)
    h1 = trace(Pb * Pdb') / trace(Pdb * Pdb');          % eq. 7
    h2 = trace(Pb * (S * Pdb)') / trace(Pdb * Pdb');
    H = [h1 -h2; h2 h1];
end
