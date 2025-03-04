% Script for selecting the parameters of the initial and desired
% configurations of each experiment.
% g0 - centroid of the rest position
% s0 - initial scale will always be 1, we're already defining the object's size above
% th0 - initial angle [radians] of the team of robots
% gd - desired centroid
% sd0 - desired scale
% thd0 - desired rotation [radians]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
switch shape

    case 'default_same_shape'  % if we want the same shape as the initial one
        g0 = [1.5 0 1.2]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 90*pi/180;

        gd = [-1.2 0 0.8]';   
        sd0 = 1;             
        thd0_x = 0*pi/180;
        thd0_y = 30*pi/180;
        thd0_z = 90*pi/180;
        
        view_1 = 24;
        view_2 = 29;
        
    case 'default_deformed_shape'  % initial deformed shape
        g0 = [1.5 0 1.2]';       
        s0 = 0.8;                 
        th0_x = 30*pi/180;
        th0_y = 180*pi/180;
        th0_z = 30*pi/180;

        gd = [-1.2 0 0.9]';   
        sd0 = 1.2;             
        thd0_x = 30*pi/180;
        thd0_y = 20*pi/180;
        thd0_z = 60*pi/180;
        
        view_1 = 24;
        view_2 = 29;
        
    case 'initial_deformed_shape'  % initial deformed shape
        g0 = [1.5 0 1.2]';       
        s0 = 0.8;                 
        th0_x = 30*pi/180;
        th0_y = 180*pi/180;
        th0_z = 45*pi/180;

        gd = [-1.2 0 0.9]';   
        sd0 = 1.2;             
        thd0_x = 30*pi/180;
        thd0_y = 20*pi/180;
        thd0_z = 60*pi/180;
        
        view_1 = 22;
        view_2 = 19;
        
    case 'bend_1'  % initial configuration bends to desired one
        g0 = [0 0 0.4]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [0 0 1.5]';   
        sd0 = 1.5;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = 0*pi/180;
        
        view_1 = 24;
        view_2 = 29;

    case 'bend_2'  % initial configuration bends to desired one
        g0 = [1.5 0 0.4]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [-0.6 0 1.7]';   
        sd0 = 1;             
        thd0_x = 0*pi/180;
        thd0_y = -45*pi/180;
        thd0_z = 30*pi/180;
        
        view_1 = 5;
        view_2 = 18;
    
    case 'bend_3'  % initial configuration bends to desired one
        g0 = [0 0 0.4]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [0 0 1.5]';   
        sd0 = 1.5;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = 0*pi/180;
        
        view_1 = 15;
        view_2 = 8;

    case 'bend_4'  % initial configuration bends to desired one
        g0 = [1.5 0 1.2]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [-0.8 0 1.8]';   
        sd0 = 1.2;             
        thd0_x = 0*pi/180;
        thd0_y = -45*pi/180;
        thd0_z = 0*pi/180;
        
        view_1 = 18;
        view_2 = 13;

    case 'bend_5'  % initial configuration bends to desired one
        g0 = [0 0 1]';       
        s0 = 1.2;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [0 0 2.5]';   
        sd0 = 1;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = 0*pi/180;
        
        view_1 = 24;
        view_2 = 29;
    
    case 'bend_6'  % initial configuration bends to desired one
        g0 = [0 0 2]';       
        s0 = 1.2;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [0 0 0.5]';   
        sd0 = 0.7;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = 0*pi/180;
        
%         view_1 = 0; % front view
%         view_2 = 0;
        
%         view_1 = 0; % top view
%         view_2 = 90;

%         view_1 = 90; % side view
%         view_2 = 0;

        view_1 = 30; % complete view
        view_2 = 35;
    
    case 'twist_1'  % initial configuration bends to desired one
        g0 = [0 0 0.4]';       
        s0 = 1;                 
        th0_x = 0*pi/180;
        th0_y = 0*pi/180;
        th0_z = 0*pi/180;

        gd = [0 0 1.5]';   
        sd0 = 1.5;             
        thd0_x = 0*pi/180;
        thd0_y = 0*pi/180;
        thd0_z = 0*pi/180;
        
        view_1 = 68;
        view_2 = 19;

    case 'inf_rotation' 
        g0 = [1.5 0 1.2]';       
        s0 = 1;                 
        th0_x = 30*pi/180;
        th0_y = 30*pi/180;
        th0_z = 30*pi/180;

        gd = [-1.2 0 0.9]';   
        sd0 = 1;             
        thd0_x = 30*pi/180;
        thd0_y = 30*pi/180;
        thd0_z = 20*pi/180;
        
        view_1 = 24;
        view_2 = 29;

end
    