% Script for defining the reference configuration, c, depending on the
% experiment we want to execute.
    
switch shape

    case 'default_same_shape' % rectangular shape as reference configuration
        c = p00;
    
    case 'default_deformed_shape' % rectangular shape as reference configuration
        c = [-1.5 -0.5 0, -0.5 -0.5 0, 0.5 -0.5 0, 1.5 -0.5 0, 1.5 0.5 0, 0.5 0.5 0, -0.5 0.5 0, -1.5 0.5 0]';
    
    case 'initial_deformed_shape' % rectangular shape as reference configuration
        c = [-1.5 -0.5 0, -0.5 -0.5 0, 0.5 -0.5 0, 1.5 -0.5 0, 1.5 0.5 0, 0.5 0.5 0, -0.5 0.5 0, -1.5 0.5 0]';
    
    case 'bend_1' % rectangular shape as reference configuration
        c = [-1.2 -0.6 -0.8, -1.2 -0.6 0, 1.2 -0.6 0, 1.2 -0.6 -0.8, 1.2 0.6 -0.8, 1.2 0.6 0, -1.2 0.6 0, -1.2 0.6 -0.8]';
    
    case 'bend_2' % rectangular shape as reference configuration
        c = [-1.2 -0.6 -0.8, -1.2 -0.6 0, 1.2 -0.6 0, 1.2 -0.6 -0.8, 1.2 0.6 -0.8, 1.2 0.6 0, -1.2 0.6 0, -1.2 0.6 -0.8]';
        
    case 'bend_3' % rectangular shape as reference configuration
        c = [-1.2 -0.6 -0.8, -1.2 -0.6 -0.4, -0.8 -0.6 0.2, 0 -0.6 0.4, 0.8 -0.6 0.2, 1.2 -0.6 -0.4, 1.2 -0.6 -0.8, ...
             1.2 0.6 -0.8, 1.2 0.6 -0.4, 0.8 0.6 0.2, 0 0.6 0.4, -0.8 0.6 0.2, -1.2 0.6 -0.4, -1.2 0.6 -0.8]';

    case 'bend_4' % rectangular shape as reference configuration
        c = [-1.2 -0.6 -0.8, -1.2 -0.6 -0.4, -0.8 -0.6 0.2, 0 -0.6 0.4, 0.8 -0.6 0.2, 1.2 -0.6 -0.4, 1.2 -0.6 -0.8, ...
             1.2 0.6 -0.8, 1.2 0.6 -0.4, 0.8 0.6 0.2, 0 0.6 0.4, -0.8 0.6 0.2, -1.2 0.6 -0.4, -1.2 0.6 -0.8]';
         
    case 'bend_5' % rectangular shape as reference configuration
        c = K * reshape(Pob0(:, handled_nodes), [3*N,1]);
        
    case 'bend_6' % rectangular shape as reference configuration
        c = K * reshape(Pob0(:, handled_nodes), [3*N,1]);
        for i = 1:N
            c(3*i-2,1) = c(3*i-2,1)*1.5;
        end

    case 'twist_1' % rectangular shape as reference configuration
        c = K * reshape(Pob0(:, handled_nodes), [3*N,1]);

    case 'inf_rotation'
%         c = [-1.5 -0.5 0, -0.5 -0.5 0, 0.5 -0.5 0, 1.5 -0.5 0, 1.5 0.5 0, 0.5 0.5 0, -0.5 0.5 0, -1.5 0.5 0]';
        c=K*p00;

end





