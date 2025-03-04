% Script for defining the reference configuration, c, depending on the
% experiment we want to execute.
    
switch shape

    case 'rectangular_prism_1' % rectangular shape as reference configuration
        c = K * p00;
        
    case 'rectangular_prism_2' % rectangular shape as reference configuration
        h_obj = 0.5; 
        c_inf = [-1.5 -0.5 0, -0.5 -0.5 0, 0.5 -0.5 0, 1.5 -0.5 0, 1.5 0.5 0, 0.5 0.5 0, -0.5 0.5 0, -1.5 0.5 0];
        c_sup = [-1.5 -0.5 h_obj, -0.5 -0.5 h_obj, 0.5 -0.5 h_obj, 1.5 -0.5 h_obj, 1.5 0.5 h_obj, 0.5 0.5 h_obj, -0.5 0.5 h_obj -1.5 0.5 h_obj];
        c = [c_inf c_sup]';
    
    case 'mattress' % mattress preserving its shape
        c = K * p00;


end





