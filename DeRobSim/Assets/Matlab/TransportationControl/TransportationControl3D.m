function [v] = TransportationControl3D(positions, destinations, kH, kG, ks, kg, kth, sd, thd, vsat)

    % ++++++++++ Optional parameters evaluation ++++++++++

    % Shape-Preserving control gain
    if ~exist('kH', 'var')
        kH = 1;        % for moving toward shape-preserving transformation
    end

    % Consistent configuration control gain
    if ~exist('kG', 'var')
        kG = 2;        % for moving toward a configuration consistent with our deformation modes
    end

    % Position control gain
    if ~exist('kg', 'var')
        kg = 0.5;      % for centroid translation
    end

    % Scale control gain
    if ~exist('ks', 'var')
        ks = 0.5;      % for scaling
    end

    % Orientation & scaling control gain
    if ~exist('kth', 'var')
        kth = 0.8;     % for scaling and orientation
    end

    % Target formation scale
    if ~exist('sd','var')
        sd = 1;
    end

    % Target formation rotation
    if ~exist('thd','var')
        thd = 0;
    end

    % Max robot velocity (in norm) allowed
    if ~exist('vsat', 'var')
        vsat = 1.5;    % m/s
    end
    


    % ++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    % ++++++++++ Properties Configuration ++++++++++

    % Number of positions and destinations
    N_pos = size(positions, 2);
    N_des = size(destinations, 2);
    assert(N_pos == N_des, 'Column size of current position (%i) and final position (%i) must be equal', N_pos, N_des);
    N = N_pos;

    % We convert the position matrices into column arrays for a better computation
    c = reshape(destinations,N*3,1);
    p = reshape(positions,N*3,1);
        

    % ---------- Kabsch Algorithm ----------
    % We define the Kabsch Algorithm Matrices that we will use in our formulation
    K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix
    S = [0 -1; 1 0]; % 90 Deg rotation matrix
    % T = kron(eye(N),S);

    % We compute the position matrices using Kabsch Algorithm

    % -- Desired --
    Pdb = reshape(K * c, [3,N]);
    gd = compute_centroid(destinations);

    % -- Current --
    Pb = reshape(K * p, [3,N]);
    g = compute_centroid(positions);
    
    % We compute the deformation matrices
    % -- Matrix C_G (deformation modes -- linear, quadratic, mixed) --
    C_G = calculate_CG(N,K,c);        
     
    % -- Pseudoinverse ---
    C_Gp = pinv(C_G); % pseudoinverse
    % -- A_G???? --
    A_G = K - C_G * C_Gp; 

    % We compute the rotation matrices
    % -- Standard desired rotation matrix --
    Rd = eul2rotm([thd thd thd], 'XYZ');
    % Rd_T = kron(eye(N), Rd);
    % -- Scale and rotation matrix (for the control) --
    H_kd = sd * Rd; % we need this matrix to control the rotation

    % We compute the final rotation based on the desired positions

    W = eye(N); % as we considere w_i = 1, W is the identity matrix
    M_k = Pdb * W * Pb'; % covariant matrix
    % We apply the Singular Value Descomposition (SVD)
    [U_k, S_k, V_k] = svd(M_k);
    
    % -- Destination Rotation matrix --
    I_k = eye(3);
    I_k(3,3) = det(V_k*U_k');
    R_k = V_k * I_k * U_k';
    
    % -- Destination Rotation angles --
    angles = rotm2eul(R_k, 'ZYX'); % in radians
    th_x = angles(3);
    th_y = angles(2);
    th_z = angles(1);
        
    % Other important parameters
    c_s = norm(Pdb, "fro")^2; % it can be compute as c_s = trace(C_b * C_b')
    % -- Current Scale factor --
    s_k = trace(Pb' * R_k * Pdb) / c_s; % current scale factor
    % -- Optimal Alignment Matrix --
    H_k = s_k * R_k;  % optimal alignment matrix
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Control terms 
    % -- shape-preserving control (UH) --
    U_H = kH * (H_k * Pdb - Pb); % shape-preserving control
    u_H = reshape(U_H, [3*N,1]);
    
    % -- Deformation Control (UG) --
    u_G = -kG * A_G * p; % deformation control
    
    % -- Scale Control (US) --
    U_s = ks * (sd - s_k) * (1/s_k) * H_k * Pdb; % scale control
    u_s = reshape(U_s, [3*N,1]);
    
    % -- Position Control (Uc) --
    u_c = kg * kron(ones(N,1), gd - g); % translation control
    
    % -- Scale and Orientation Control (UHd) --
    U_Hd = kth * (H_kd * Pdb - Pb); % integrated scale and orientation control
    u_Hd = reshape(U_Hd, [3*N,1]);
    
    %%%%%%%%%%%%%%%%%%%%%%% 
  
    % Control law
    u = u_H + u_G + u_c + u_s + u_Hd;
    
    % Velocity saturation
    for i = 1:N
        if norm(u(3*i-2:3*i,:)) > vsat
            u(3*i-2:3*i,:) = vsat * u(3*i-2:3*i,:) / norm(u(3*i-2:3*i,:));
        end
    end

    v = reshape(u,3,N); 
 

end

function [g] = compute_centroid(P)
    N = size(P, 2);
    g = (1/N) * P * ones(N,1);
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