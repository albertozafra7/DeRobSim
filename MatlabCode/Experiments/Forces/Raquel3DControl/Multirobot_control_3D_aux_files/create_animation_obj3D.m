% We define the pairs of robots for which we will plot a line for each
% experiment

switch shape

    case 'rectangular_prism_1'
        N_inf = size(p_inf,2)/3; % number of robots below
        N_sup = size(p_sup,2)/3; % number of robots above
        pairs = [];
        for i = 1:((N/2)-1)
            pairs = [pairs; [i i+1]];
        end
            pairs = [pairs; [1 (N/2)]];
        for i = ((N/2)+1):(N-1)
            pairs = [pairs; [i i+1]];
        end
            pairs = [pairs; [((N/2)+1) N]];
        for i = 1:(N/2)
            pairs = [pairs; [i i+(N/2)]];
        end
        
    case 'rectangular_prism_2'
        N_inf = size(p_inf,2)/3; % number of robots below
        N_sup = size(p_sup,2)/3; % number of robots above
        pairs = [];
        for i = 1:((N/2)-1)
            pairs = [pairs; [i i+1]];
        end
            pairs = [pairs; [1 (N/2)]];
        for i = ((N/2)+1):(N-1)
            pairs = [pairs; [i i+1]];
        end
            pairs = [pairs; [((N/2)+1) N]];
        for i = 1:(N/2)
            pairs = [pairs; [i i+(N/2)]];
        end
        pairs = [pairs; [2 7]];
        pairs = [pairs; [3 6]];
        pairs = [pairs; [10 15]];
        pairs = [pairs; [11 14]];
    
    case 'mattress'
        pairs = [];
        for i = 1:N-1
            pairs = [pairs; [i i+1]];
        end
            pairs = [pairs; [1 N/2+1]; [N/2 N]; [1 N]]; % close the shape in the sheet object
        
end
