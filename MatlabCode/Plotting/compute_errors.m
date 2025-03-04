function [gamma, eg, es, eth] = compute_errors(positions, destinations, sd, thd)
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

    % -- Current --
    g = compute_centroid(P);  % Current formation centroid
    Pb = compute_relativePos(P,g);     % eq. 4
    [H, h1, h2] = compute_H(Pb,Pdb,S);

    % -- Errors --
    gamma = (1/2) * norm(Pb - H * Pdb, 'fro').^2;       % eq. 3 % Deformation difference
    s = sqrt(det(H));           % Current scale of the formation
    th = atan2(h2, h1);         % Current rotation of the formation
    eg = g - gd;                % Position error
    es = s - sd;                % Scale error
    eth = th - thd;             % Rotation error
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