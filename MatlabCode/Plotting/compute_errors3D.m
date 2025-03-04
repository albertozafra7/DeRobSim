function [gamma_H, gamma_G, eg, es, eth, eth_individual] = compute_errors3D(positions, destinations, sd, thd)
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
    c = reshape(destinations,N*3,1);

    % -- Current --
    g = compute_centroid(P);  % Current formation centroid
    Pb = compute_relativePos(P,g);     % eq. 4
    p = reshape(positions,N*3,1);

    % -- Kabsch Matrices --
    [H_k, R_k, s_k] = compute_Hk(Pb,Pdb);
    Rd = eul2rotm([thd thd thd], 'XYZ');
    A_G = compute_AG(destinations);

    % -- Errors --
    gamma_H = (1/2) * norm((Pb - H_k * Pdb), "fro")^2;
    gamma_G = (1/2) * p' * A_G * p;
    eg = norm(g-gd);                % centroid error
    es = s_k - sd;                  % scale error
    
    e_H = eye(3) - R_k\Rd;
    eth = norm(e_H, "fro");         % rotation error


    % -- Destination Rotation angles --
    angles = rotm2eul(R_k, 'ZYX'); % in radians
    th_x = angles(3);
    th_y = angles(2);
    th_z = angles(1);

    eth_x = abs(th_x - thd);
    eth_y = abs(th_y - thd);
    eth_z = abs(th_z - thd);

    eth_individual = [eth_x eth_y eth_z];
end

function [g] = compute_centroid(P)
    N = size(P, 2);
    g = (1/N) * P * ones(N,1);
end

function [Pb] = compute_relativePos(P,g)
    N = size(P, 2);
    Pb = P - g * ones(1,N); % eq. 5
end

function [H_k, R_k, s_k] = compute_Hk(Pb,Pdb)
    N = size(Pb,2);
    W = eye(N); % as we considere w_i = 1, W is the identity matrix
    M_k = Pdb * W * Pb'; % covariant matrix
    % We apply the Singular Value Descomposition (SVD)
    [U_k, ~, V_k] = svd(M_k);
    
    % -- Destination Rotation matrix --
    I_k = eye(3);
    I_k(3,3) = det(V_k*U_k');
    R_k = V_k * I_k * U_k';
    
    c_s = norm(Pdb, "fro")^2; % it can be compute as c_s = trace(C_b * C_b')
    s_k = trace(Pb' * R_k * Pdb) / c_s; % current scale factor
    H_k = s_k * R_k;  % optimal alignment matrix
end

function [A_G] = compute_AG(destinations)
    N = size(destinations, 2);
    c = reshape(destinations,N*3,1);

    % ---------- Kabsch Algorithm ----------
    % We define the Kabsch Algorithm Matrices that we will use in our formulation
    K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix

    % We compute the deformation matrices
    % -- Matrix C_G (deformation modes -- linear, quadratic, mixed) --
    C_G = calculate_CG(N,K,c);        
     
    % -- Pseudoinverse ---
    C_Gp = pinv(C_G); % pseudoinverse
    % -- A_G???? --
    A_G = K - C_G * C_Gp; 
end

function C_G = calculate_CG(N,K,c)
    
% Initialization of each parameter
    l1 = zeros(3*N,1); % l: linear
    l2 = zeros(3*N,1);
    l3 = zeros(3*N,1);
    l4 = zeros(3*N,1);
    l5 = zeros(3*N,1);
    l6 = zeros(3*N,1);
    l7 = zeros(3*N,1);
    l8 = zeros(3*N,1);
    l9 = zeros(3*N,1);
    q1 = zeros(3*N,1); % q: quadratic
    q2 = zeros(3*N,1);
    q3 = zeros(3*N,1);
    q4 = zeros(3*N,1);
    q5 = zeros(3*N,1);
    q6 = zeros(3*N,1);
    q7 = zeros(3*N,1);
    q8 = zeros(3*N,1);
    q9 = zeros(3*N,1);
    m1 = zeros(3*N,1); % m: mixed
    m2 = zeros(3*N,1);
    m3 = zeros(3*N,1);
    m4 = zeros(3*N,1);
    m5 = zeros(3*N,1);
    m6 = zeros(3*N,1);
    m7 = zeros(3*N,1);
    m8 = zeros(3*N,1);
    m9 = zeros(3*N,1);

    for i = 1:N

        l1(3*i-2) = c(3*i-2);
        l2(3*i-2) = c(3*i-1);
        l3(3*i-2) = c(3*i);
        l4(3*i-1) = c(3*i-2);
        l5(3*i-1) = c(3*i-1);
        l6(3*i-1) = c(3*i);
        l7(3*i) = c(3*i-2);
        l8(3*i) = c(3*i-1);
        l9(3*i) = c(3*i);

        q1(3*i-2) = c(3*i-2).^2;
        q2(3*i-2) = c(3*i-1).^2;
        q3(3*i-2) = c(3*i).^2;
        q4(3*i-1) = c(3*i-2).^2;
        q5(3*i-1) = c(3*i-1).^2;
        q6(3*i-1) = c(3*i).^2;
        q7(3*i) = c(3*i-2).^2;
        q8(3*i) = c(3*i-1).^2;
        q9(3*i) = c(3*i).^2;

        m1(3*i-2) = c(3*i-2) * c(3*i-1);
        m2(3*i-2) = c(3*i-1) * c(3*i);
        m3(3*i-2) = c(3*i) * c(3*i-2);
        m4(3*i-1) = c(3*i-2) * c(3*i-1);
        m5(3*i-1) = c(3*i-1) * c(3*i);
        m6(3*i-1) = c(3*i) * c(3*i-2);
        m7(3*i) = c(3*i-2) * c(3*i-1);
        m8(3*i) = c(3*i-1) * c(3*i);
        m9(3*i) = c(3*i) * c(3*i-2);

    end
    
    C_G = K*[l1, l2, l3, l4, l5, l6, l7, l8, l9, ...
             q1, q2, q3, q4, q5, q6, q7, q8, q9, ...
             m1, m2, m3, m4, m5, m6, m7, m8, m9];  

end