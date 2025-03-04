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