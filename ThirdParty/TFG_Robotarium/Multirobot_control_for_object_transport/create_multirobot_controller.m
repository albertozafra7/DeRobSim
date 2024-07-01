% Control of a team of mobile robots in a 2D working area with double-integrator dynamics.

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
% Modified by: Raquel Marcos, September 2022

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ multirobot_controller_for_obj_trans ] = create_multirobot_controller(varargin)

    parser = inputParser;    

%         % Gains of the controllers. These are the values chosen in the ICRA 22 paper, which may not be the best
%         % ones in all cases.
%         parser.addOptional('k1H', 4);
%         parser.addOptional('k2H', 2);
%         parser.addOptional('k1G', 4);
%         parser.addOptional('k2G', 2);
%         parser.addOptional('k1s', 3);
%         parser.addOptional('k2s', 1);
%         parser.addOptional('k1gm', 0.2);
%         parser.addOptional('k2gm', 1);
%         parser.addOptional('k1th', 3);
%         parser.addOptional('k2th', 1);
%         parser.addOptional('alpha_H', 1);
%         parser.addOptional('alpha_G', 10);
        
    % Different set of gain values:
    parser.addOptional('k1H', 6);
    parser.addOptional('k2H', 3);
    parser.addOptional('k1G', 3);
    parser.addOptional('k2G', 2);
    parser.addOptional('k1s', 5);
    parser.addOptional('k2s', 6);
    parser.addOptional('k1gm', 2);
    parser.addOptional('k2gm', 6);
    parser.addOptional('k1th', 4);
    parser.addOptional('k2th', 4);
    parser.addOptional('alpha_H', 4);
    parser.addOptional('alpha_G', 2);

    % Other variables the controller will use.
    parser.addOptional('dt', 0.033);      % Time step of the simulation
    parser.addOptional('sd', 1);          % Target formation scale
    parser.addOptional('thd', 0);         % Target formation rotation
    parser.addOptional('asat', 1e10);     % Max robot acceleration (in norm) allowed
    
    parser.parse(varargin{:});  % If we want to change the value of any variable, we can do it when calling the controller.

    % We create real variables that can be used by the controller.
    k1H = parser.Results.k1H;
    k2H = parser.Results.k2H;
    k1G = parser.Results.k1G;
    k2G = parser.Results.k2G;
    k1s = parser.Results.k1s;
    k2s = parser.Results.k2s;
    k1g = parser.Results.k1gm;
    k2g = parser.Results.k2gm;
    k1th = parser.Results.k1th;
    k2th = parser.Results.k2th;
    alpha_H = parser.Results.alpha_H;
    alpha_G = parser.Results.alpha_G;
    dt = parser.Results.dt;             
    sd = parser.Results.sd;
    thd = parser.Results.thd;
    asat = parser.Results.asat;         

    S = [0 -1; 1 0];    % 90 deg rotation matrix to be used in the control

    % Initialize stored values from previous iteration
    prev_P = 0;
    prev_Pb = 0;
    prev_H = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    multirobot_controller_for_obj_trans = @multirobot_controller_for_obj_trans_;
    
    function [ a, data ] = multirobot_controller_for_obj_trans_(positions, destinations)
        
        N_pos = size(positions, 2);
        N_des = size(destinations, 2);
        assert(N_pos == N_des, 'Column size of current position (%i) and final position (%i) must be equal', N_pos, N_des);

        N = N_pos;          % Number of robots
        P = positions;
        Pd = destinations;

        gd = (1/N) * Pd * ones(N,1);
        Pdb = Pd - gd * ones(1,N); % eq. 5
        Pdbp = pinv(Pdb); % Pseudoinverse
        
        % We first compute the variables we will need for the controllers
        g = (1/N) * P * ones(N,1);  % Current formation centroid
        Pb = P - g * ones(1,N);     % eq. 4
        h1 = trace(Pb * Pdb') / trace(Pdb * Pdb');          % eq. 7
        h2 = trace(Pb * (S * Pdb)') / trace(Pdb * Pdb');
        H = [h1 -h2; h2 h1];
        gamma = (1/2) * norm(Pb - H * Pdb, 'fro').^2;       % eq. 3
        s = sqrt(det(H));           % Current scale of the formation
        th = atan2(h2, h1);         % Current rotation of the formation
        eg = g - gd;                % Position error
        es = s - sd;                % Scale error
        eth = th - thd;             % Rotation error

        % We save the following variables in 'data' in order to use them in
        % the main script
        data.gamma = gamma;
        data.H = H;
        data.P = P;
        data.Pb = Pb;
        data.eg = norm(eg);         % Use norm because it is not a scalar
        data.es = es;
        data.eth = eth;

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

        % We save P, Pb and H values in this iteration for using in the next one
        prev_P = P;
        prev_Pb = Pb;
        prev_H = H;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % We compute the different controllers (i.e., the additive terms that make up
        % the full control law) using the variables we have computed above:
    
        U_H = k1H * (H * Pdb - Pb) + k2H * (Hdot * Pdb - Pbdot);                    % eq. 8

        U_G = k1G * (Pb * Pdbp * Pdb - Pb) + k2G * (Pbdot * Pdbp * Pdb - Pbdot);    % eq. 9
        
        U_gamma = alpha_H * U_H + alpha_G * U_G;                                    % eq. 10
    
        U_s = -k1s * es * (1/s) * H * Pdb - k2s * Pbdot;                            % eq. 17
    
        U_D = U_gamma + U_s;                                                        % eq. 18
    
        U_g = -k1g * eg * ones(1,N) - k2g * Pdot;                                   % eq. 19
    
        U_th = -k1th * eth * S * H * Pdb - k2th * Pbdot;                            % eq. 20
        
        U_f = U_D + U_g + U_th;                                                     % full control, eq. 22
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % We move the robots.        
        a = U_f;    % Acceleration to be applied for the computed full control

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

        % We store the norms of acceleration
        for i = 1:N 
            na(i) = norm(a(:,i));
        end

        % We save the following variables in 'data' in order to use them in
        % the main script
        data.Pdot = Pdot;
        data.Pbdot = Pbdot;
        data.Hdot = Hdot;
        data.a = a;
        data.na = na;

    end
end