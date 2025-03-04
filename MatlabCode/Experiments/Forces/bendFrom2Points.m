function [deformed_vertices] = bendFrom2Points(original_vertices,bending_points, bend_angle, bending_axis)
    % Copy the original vertices
    x = original_vertices(:,1);
    y = original_vertices(:,2);
    z = original_vertices(:,3);

    % Midpoint between grabbing points (neutral axis of bending)
    neutral_axis_x = (bending_points(1,1) + bending_points(2,1)) / 2;
    
    % Radius of curvature for bending
    distance_between_bend_points = norm(bending_points(1,:) - bending_points(2,:));
    bend_radius = distance_between_bend_points / (2 * sin(bend_angle / 2));
    
    % Calculate angular displacement for each vertex based on its x-position
    relative_x = x - neutral_axis_x; % Relative x-position from the neutral axis
    theta = relative_x / bend_radius; % Angular displacement
    
    % Apply circular bending deformation
    if bending_axis == 'z'
        % Compute the new x and z coordinates (circular arc in the XZ-plane)
        new_x = neutral_axis_x + bend_radius * sin(theta); % New x positions
        new_z = z + bend_radius * (1 - cos(theta));        % New z positions
        deformed_vertices = [new_x, y, new_z];            % Keep y unchanged
    elseif bending_axis == 'y'
        % Compute the new x and y coordinates (circular arc in the XY-plane)
        new_x = neutral_axis_x + bend_radius * sin(theta); % New x positions
        new_y = y + bend_radius * (1 - cos(theta));        % New y positions
        deformed_vertices = [new_x, new_y, z];            % Keep z unchanged
    else
        error('Unsupported bending axis');
    end
end