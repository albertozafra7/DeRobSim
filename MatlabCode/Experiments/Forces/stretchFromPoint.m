function [deformed_vertices] = stretchFromPoint(original_vertices, grabbing_point, fixed_points, poisson_ratio, stretch_amount)

    % Copy the original vertices
    x = original_vertices(:,1);
    y = original_vertices(:,2);
    z = original_vertices(:,3);


    % Compute distances from each vertex to the grabbing point
    distance_to_grab = sqrt((x - grabbing_point(1)).^2 + ...
                            (y - grabbing_point(2)).^2 + ...
                            (z - grabbing_point(3)).^2);
    
    % Normalize the distances to create weights (closer = higher deformation)
    max_distance_to_grab = max(distance_to_grab);
    weights = 1 - (distance_to_grab / max_distance_to_grab);  % Normalize to [0, 1]
    
    % Ensure vertices close to the fixed points do not deform
    distances_to_fixed = zeros(size(original_vertices, 1), 3);
    for i = 1:length(fixed_points)
        distances_to_fixed(:, i) = sqrt((x - fixed_points(i, 1)).^2 + ...
                                        (y - fixed_points(i, 2)).^2 + ...
                                        (z - fixed_points(i, 3)).^2);
    end
    min_distance_to_fixed = min(distances_to_fixed, [], 2);
    
    % Combine weights: vertices near fixed points have less deformation
    weights = weights .* (min_distance_to_fixed / max(min_distance_to_fixed));
    
    % Stretch along the X-axis
    x_stretched = x + stretch_amount * weights .* (x - grabbing_point(1)) ./ max(abs(x - grabbing_point(1)));
    
    % Contract in the Y-direction proportional to the X stretch
    y_contracted = y - poisson_ratio * (stretch_amount * weights .* (y - grabbing_point(2)) ./ max(abs(y - grabbing_point(2))));
    
    % Combine deformed coordinates
    deformed_vertices = [x_stretched, y_contracted, z];
end