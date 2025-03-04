% Script for selecting the parameters of the initial and desired
% configurations of each experiment.
% h_obj - object's height
% p_inf - init. conf. face below
% p_sup - init. conf. face above
% g0 - centroid of the rest position
% s0 - initial scale will always be 1, we're already defining the object's size above
% th0 - initial angle [radians] of the team of robots
% gd - desired centroid
% sd0 - desired scale
% thd0 - desired rotation [radians]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
switch shape

    case 'rectangular_prism_1'
        h_obj = 0.5; 
        p_inf = [-1.5 -0.5 0, 1.5 -0.5 0, 1.5 0.5 0, -1.5 0.5 0];
        p_sup = [-1.5 -0.5 h_obj, 1.5 -0.5 h_obj, 1.5 0.5 h_obj, -1.5 0.5 h_obj];
        p_init = [p_inf p_sup]';
        
        g0 = [1.5 0 1.2]';       
        s0 = 0.6;                 
        th0_x = 30*pi/180;
        th0_y = 60*pi/180;
        th0_z = 30*pi/180;

        gd = [-1.2 0 0.8]';   
        sd0 = 1;             
        thd0_x = 30*pi/180;
        thd0_y = 20*pi/180;
        thd0_z = 30*pi/180;
        
        view_1 = 24;
        view_2 = 29;

    case 'rectangular_prism_2' % bend
        h_obj = 0.5; 
        p_inf = [-1.5 -0.5 1, -0.5 -0.5 0, 0.5 -0.5 0, 1.5 -0.5 1, 1.5 0.5 1, 0.5 0.5 0, -0.5 0.5 0, -1.5 0.5 1];
        p_sup = [-1.5 -0.5 1+h_obj, -0.5 -0.5 h_obj, 0.5 -0.5 h_obj, 1.5 -0.5 1+h_obj, 1.5 0.5 1+h_obj, 0.5 0.5 h_obj, -0.5 0.5 h_obj, -1.5 0.5 1+h_obj];
        p_init = [p_inf p_sup]';
        
        g0 = [1.5 0 1.2]';       
        s0 = 0.6;                 
        th0_x = 30*pi/180;
        th0_y = 60*pi/180;
        th0_z = 30*pi/180;

        gd = [-1.2 0 0.8]';   
        sd0 = 1;             
        thd0_x = 30*pi/180;
        thd0_y = 20*pi/180;
        thd0_z = 30*pi/180;
        
        view_1 = 24;
        view_2 = 29;
        
    case 'mattress' % 6 robots
        p_init = [   0    0 0.15,...
                     0 1.05 0.15,...
                     0  2.1 0.15,...
                   1.5    0 0.15,...
                   1.5 1.05 0.15,...
                   1.5  2.1 0.15]';        
        g0 = [0.75 1.05 0.15]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [-1.95 1.05 0.15]';
        gd_final = gd; % it is neccesary for the waypoints
        sd0 = 1;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = -180*pi/180;
        
        view_1 = 110;
        view_2 = 29;
        

        

end
    