function [times, positions, accels, vels, UD, UG, UH, Ugamma, Us, Ug, Uth, Uf, gamma_err, eg_err, es_err, eth_err] = simulateControl2D(origins, destinations, time_end, timestep, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat)
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
    
    % ++++++++++ Returning values creation ++++++++++
    times = 0:timestep:time_end;
    n_steps = length(times);
    n_agents = length(origins);

    % Positions storing
    positions.x = zeros(n_steps,n_agents);
    positions.y = zeros(n_steps,n_agents);
    positions.z = zeros(n_steps,n_agents);

    % Accelerations storing
    accels.x = zeros(n_steps,n_agents);
    accels.y = zeros(n_steps,n_agents);
    accels.z = zeros(n_steps,n_agents);

    % Velocity storing
    vels.x = zeros(n_steps,n_agents);
    vels.y = zeros(n_steps,n_agents);
    vels.z = zeros(n_steps,n_agents);

    % UD
    UD.x = zeros(n_steps,n_agents);
    UD.y = zeros(n_steps,n_agents);
    UD.z = zeros(n_steps,n_agents);

    % UG
    UG.x = zeros(n_steps,n_agents);
    UG.y = zeros(n_steps,n_agents);
    UG.z = zeros(n_steps,n_agents);

    % UH
    UH.x = zeros(n_steps,n_agents);
    UH.y = zeros(n_steps,n_agents);
    UH.z = zeros(n_steps,n_agents);

    % Ugamma
    Ugamma.x = zeros(n_steps,n_agents);
    Ugamma.y = zeros(n_steps,n_agents);
    Ugamma.z = zeros(n_steps,n_agents);

    % Us
    Us.x = zeros(n_steps,n_agents);
    Us.y = zeros(n_steps,n_agents);
    Us.z = zeros(n_steps,n_agents);

    % Ug
    Ug.x = zeros(n_steps,n_agents);
    Ug.y = zeros(n_steps,n_agents);
    Ug.z = zeros(n_steps,n_agents);

    % Uth
    Uth.x = zeros(n_steps,n_agents);
    Uth.y = zeros(n_steps,n_agents);
    Uth.z = zeros(n_steps,n_agents);

    % Uf
    Uf.x = zeros(n_steps,n_agents);
    Uf.y = zeros(n_steps,n_agents);
    Uf.z = zeros(n_steps,n_agents);

    % gamma
    gamma_err = zeros(n_steps,1);
    % eg
    eg_err = zeros(n_steps,1);
    % es
    es_err = zeros(n_steps,1);
    % eth
    eth_err = zeros(n_steps,1);
    

    % ++++++++++ Simulation ++++++++++
    % Data Preparation
    agentPose = origins;
    agentPrevPose = origins;
    velocities = zeros(2,n_agents);
    

    for t = 1:n_steps
        % ---- Perform the control ----
        [ acelerations, U_f, U_D, U_g, U_th, U_gamma, U_s, U_H, U_G, ~, ~, ~, gamma, eg, es, eth ] = TransportationControl2D_Debug(agentPose, destinations, agentPrevPose, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
        [poses,velocities] = moveAgent(agentPose,velocities,acelerations,timestep);
        agentPrevPose = agentPose;
        agentPose = poses;

        % ---- Save the control values ----
        % This is equivalent to first save the poses and the move (as Unity)
        positions.x(t,:) = agentPrevPose(1,:);
        %positions.y(t,:) = agentPrevPose(2,:);
        positions.z(t,:) = agentPrevPose(end,:);

        vels.x(t,:) = velocities(1,:);
        %vels.y(t,:) = comp_vels(2,:);
        vels.z(t,:) = velocities(end,:);

        accels.x(t,:) = acelerations(1,:);
        %accels.y(t,:) = acelerations(2,:);
        accels.z(t,:) = acelerations(end,:);

        Uf.x(t,:) = U_f(1,:);
        % Uf.y(t,:) = U_f(2,:);
        Uf.z(t,:) = U_f(end,:);

        UD.x(t,:) = U_D(1,:);
        % UD.y(t,:) = U_D(2,:);
        UD.z(t,:) = U_D(end,:);

        Ug.x(t,:) = U_g(1,:);
        % Ug.y(t,:) = U_g(2,:);
        Ug.z(t,:) = U_g(end,:);

        Uth.x(t,:) = U_th(1,:);
        % Uth.y(t,:) = U_th(2,:);
        Uth.z(t,:) = U_th(end,:);

        Ugamma.x(t,:) = U_gamma(1,:);
        % Ugamma.y(t,:) = U_gamma(2,:);
        Ugamma.z(t,:) = U_gamma(end,:);

        Us.x(t,:) = U_s(1,:);
        % Us.y(t,:) = U_s(2,:);
        Us.z(t,:) = U_s(end,:);

        UH.x(t,:) = U_H(1,:);
        % UH.y(t,:) = U_H(2,:);
        UH.z(t,:) = U_H(end,:);

        Us.x(t,:) = U_s(1,:);
        % Us.y(t,:) = U_s(2,:);
        Us.z(t,:) = U_s(end,:);

        UG.x(t,:) = U_G(1,:);
        % UG.y(t,:) = U_G(2,:);
        UG.z(t,:) = U_G(end,:);

        gamma_err(t) = gamma;
        eg_err(t) = norm(eg);
        es_err(t) = es;
        eth_err(t) = eth;
    end
end

% Function for simulating the agent movement
function [poses,vels] = moveAgent(positions, velocities, accelerations,dt)
    vels = velocities + accelerations*dt;
    poses = positions + vels*dt;
end