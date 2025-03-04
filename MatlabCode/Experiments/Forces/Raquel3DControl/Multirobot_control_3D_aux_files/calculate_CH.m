% Matrix C_H has four columns.
% First column, c1: Terms that multiply h_s
% Second column, c2: Terms that multiply h_rx
% Third column, c3: Terms that multiply h_ry
% Fourth column, c4: Terms that multiply h_rz

function C_H = calculate_CH(N,K,c)
    
    % Initialization of the three last columns
    c2 = zeros(3*N, 1);
    c3 = zeros(3*N, 1);
    c4 = zeros(3*N, 1);
    
    % We compute the terms of each column
    c1 = K * c;
    
    for i = 1:N
        c2(3*i-1 : 3*i) = [-c1(3*i); c1(3*i-1)];
        c3(3*i-2 : 2 : 3*i) = [c1(3*i); -c1(3*i-2)];
        c4(3*i-2 : 3*i-1) = [-c1(3*i-1); c1(3*i-2)];
    end
    
    C_H = K * [c1 c2 c3 c4];

end