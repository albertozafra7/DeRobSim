function vertices = generateVertices(shape, params, N_vertices)
    switch shape
        case 'cylinder'
            % Cylinder parameters
            outer_radius = params.radius;
            height = params.height;
            num_faces = 2;            % Number of faces (top and bottom)
            vertices_per_face = 248;  % Total vertices per face
            num_circles = 8;          % Number of concentric circles per face
            
            % Proportional vertex distribution
            circle_radii = linspace(outer_radius / num_circles, outer_radius, num_circles); % Radii of circles
            circumferences = 2 * pi * circle_radii;  % Circumference of each circle
            vertex_distribution = circumferences / sum(circumferences) * vertices_per_face; % Proportional distribution
            vertex_counts = round(vertex_distribution);  % Round to integers
            
            % Adjust to match the total number of vertices exactly
            adjustment = vertices_per_face - sum(vertex_counts);
            vertex_counts(end) = vertex_counts(end) + adjustment; % Adjust the last circle's count
            
            % Initialize vertex storage
            vertices = [];
            z_positions = [-height/2, height/2]; % Bottom and top face z-coordinates
            
            % Generate vertices for each face
            for z = z_positions
                for i = 1:num_circles
                    % Radius of the current circle
                    r = circle_radii(i);
                    % Number of vertices for this circle
                    n_points = vertex_counts(i);
                    % Generate circular points
                    theta = linspace(0, 2*pi, n_points + 1); % Add one extra to close the loop
                    theta = theta(1:end-1);                 % Remove duplicate at 2*pi
                    x = r * cos(theta);
                    y = r * sin(theta);
                    circle_vertices = [x', y', repmat(z, n_points, 1)];
                    vertices = [vertices; circle_vertices];
                end
            end
            vertices = [vertices;0 0 0];


        case 'sphere'
            % Sphere parameters
            radius = params.radius;
            phi = linspace(0, pi, round(sqrt(N_vertices))); % Latitude
            theta = linspace(0, 2*pi, round(N_vertices / length(phi))); % Longitude

            % Compute vertices
            [Phi, Theta] = meshgrid(phi, theta);
            X = radius * sin(Phi) .* cos(Theta);
            Y = radius * sin(Phi) .* sin(Theta);
            Z = radius * cos(Phi);
            vertices = [X(:), Y(:), Z(:)];

        case 'cube'
            % Cube parameters
            side_length = params.side_length;
            grid_points = round(nthroot(N_vertices, 3));
            points = linspace(-side_length/2, side_length/2, grid_points);

            % Compute vertices
            [X, Y, Z] = ndgrid(points, points, points);
            vertices = [X(:), Y(:), Z(:)];

        otherwise
            error('Shape not supported!');
    end

    % Truncate vertices if needed
    % vertices = vertices(1:N_vertices, :);
end
