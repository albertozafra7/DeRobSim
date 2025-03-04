function DeformableObjectGUI
% Create main figure
    fig = uifigure('Name', 'Deformable Object GUI', 'Position', [100 100 800 600]);
    
    % Create 3D axes
    ax = uiaxes(fig, 'Position', [50 200 700 400], 'Box', 'on', 'GridLineStyle', '-');
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    ax.ZGrid = 'on';
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    
    % +++ Create UI components and store handles +++
    handles = struct();
    
    % Fix Selected Button
    handles.fixButton = uibutton(fig, 'Position', [25 100 100 22], 'Text', 'Fix Selected', 'ButtonPushedFcn', @toggleFixed);
    
    % Force Applier fields
    handles.forceXEdit = uieditfield(fig, 'numeric', 'Position', [150 80 60 22], 'Value', 0);
    handles.forceYEdit = uieditfield(fig, 'numeric', 'Position', [220 80 60 22], 'Value', 0);
    handles.forceZEdit = uieditfield(fig, 'numeric', 'Position', [290 80 60 22], 'Value', 0);
    handles.applyForceButton = uibutton(fig, 'Position', [360 80 80 22], 'Text', 'Apply Force', 'ButtonPushedFcn', @applyForce);
    
    % Object Displacement fields
    handles.moveXEdit = uieditfield(fig, 'numeric', 'Position', [150 150 60 22], 'Value', 0);
    handles.moveYEdit = uieditfield(fig, 'numeric', 'Position', [220 150 60 22], 'Value', 0);
    handles.moveZEdit = uieditfield(fig, 'numeric', 'Position', [290 150 60 22], 'Value', 0);
    handles.moveButton = uibutton(fig, 'Position', [360 150 80 22], 'Text', 'Move All', 'ButtonPushedFcn', @moveObject);
    
    % Reset Button
    handles.resetButton = uibutton(fig, 'Position', [590 80 80 22], 'Text', 'Reset', 'ButtonPushedFcn', @resetObject);
    
    % Selection Threshold
    handles.thresholdEdit = uieditfield(fig, 'numeric', 'Position', [20 130 110 22], 'Value', 0.2, 'ValueChangedFcn', @updateThreshold);
    
    % Young Modulus
    handles.youngEdit = uieditfield(fig, 'numeric', 'Position', [460 140 110 22], 'Value', 210e10, 'ValueChangedFcn', @updateYoungModulus);
    
    % Poisson Ratio
    handles.poissonEdit = uieditfield(fig, 'numeric', 'Position', [460 80 110 22], 'Value', 0.3, 'ValueChangedFcn', @updatePoisson);
    
    % Load Data Button
    handles.loadButton = uibutton(fig, 'Position', [590 160 80 22], 'Text', 'Load Data', 'ButtonPushedFcn', @loadData);
    
    % Initial Positions Slider
    handles.initialSwitch = uiswitch(fig, 'slider', 'Items', ["off","on"], 'Value', "on", 'Position', [720 80 40 20], 'ValueChangedFcn', @toggleInitialPositions);
    
    % Von Mises Stress Slider
    handles.vmSwitch = uiswitch(fig, 'slider', 'Items', ["off", "on"], 'Value', "off", 'Position', [720, 140, 40, 20], 'ValueChangedFcn', @toggleVonMises);
    
    % Record Jacobian Button
    handles.recordJButton = uibutton(fig, 'Position', [590 115 80 22], 'Text', 'Record J', 'ButtonPushedFcn', @recordJacobian);

    % Labels
    handles.forceXLabel = uilabel(fig, 'Position', [150 100 60 22], 'Text', 'Force X:');
    handles.forceYLabel = uilabel(fig, 'Position', [220 100 60 22], 'Text', 'Force Y:');
    handles.forceZLabel = uilabel(fig, 'Position', [290 100 60 22], 'Text', 'Force Z:');
    handles.moveXLabel = uilabel(fig, 'Position', [150 170 60 22], 'Text', 'Move X:');
    handles.moveYLabel = uilabel(fig, 'Position', [220 170 60 22], 'Text', 'Move Y:');
    handles.moveZLabel = uilabel(fig, 'Position', [290 170 60 22], 'Text', 'Move Z:');
    handles.thresholdLabel = uilabel(fig, 'Position', [20 150 120 22], 'Text', 'Selection Threshold:');
    handles.youngLabel = uilabel(fig, 'Position', [460 160 120 22], 'Text', 'Young Modulus:');
    handles.poissonLabel = uilabel(fig, 'Position', [460 100 120 22], 'Text', 'Poisson Ratio:');
    handles.loadLabel = uilabel(fig, 'Position', [590 140 120 22], 'Text', 'No file loaded');
    handles.initialLabel = uilabel(fig, 'Position', [690 100 120 22], 'Text', 'Show Initial Pose');
    handles.vmLabel = uilabel(fig, 'Position', [690 160 120 22], 'Text', 'Show von Mises');
    
    % Separation Lines
    handles.line1 = uicontrol(fig, 'Style', 'text', 'Position', [140 80 1 100], 'BackgroundColor', 'k');
    handles.line2 = uicontrol(fig, 'Style', 'text', 'Position', [450 80 1 100], 'BackgroundColor', 'k');
    handles.line3 = uicontrol(fig, 'Style', 'text', 'Position', [580 80 1 100], 'BackgroundColor', 'k');
    handles.line4 = uicontrol(fig, 'Style', 'text', 'Position', [680 80 1 100], 'BackgroundColor', 'k');
    
    % +++ Particles and Data Initialization +++

    nx = 9; % number of nodes in x direction of the grid
    ny = 13;% numbe of nodes in y direction of the grid
    nz = 3; % number of nodes in z direction of the grid
    
    sx = 4; % length of the object in x direction (in the graphics, it will be the 'y' direction)
    sy = 6; % length of the object in y direction (only relevant if ob_type = 1) (in the graphics, it will be the x direction)
    sz = 2; % length of the object in z direction
    
    % We need to reduce the defined object for the Robotarium experiment because its
    % working area is smaller.
    scale_ob = 0.3;
    omesh = create_mesh3D(nx, ny, nz, sx * scale_ob, sy * scale_ob, sz * scale_ob); % mesh of the object: its shape and the links between nodes
    % We put the object over the Z axis (which means that the lowest part of
    % the body will be at 0 or over, regarding the Z axis, so it does not go
    % under the floor
    omesh.shape = omesh.shape + [0, 0, -1*(min(omesh.shape(:,3))<0)*min(omesh.shape(:,3))];
    X = omesh.shape(:,1);
    Y = omesh.shape(:,2);
    Z = omesh.shape(:,3);


    % [X, Y, Z] = meshgrid(-2:2, -2:2, -2:2);
    initialPositions = [X(:), Y(:), Z(:)];
    nParticles = size(initialPositions, 1);
    selectionThreshold = 0.2;


    % Get the triangles and tetrahedra of the object
    shp = alphaShape(X(:), Y(:), Z(:));
    surfTriangles = boundaryFacets(shp);
    gm = fegeometry([X(:), Y(:), Z(:)], surfTriangles);
    model = femodel(AnalysisType="structuralStatic", Geometry=gm);
    model.MaterialProperties = materialProperties(YoungsModulus=210e10, PoissonsRatio=0.3, MassDensity=2.7e-6);
    model = generateMesh(model, GeometricOrder="linear");
    initialTetPositions = model.Geometry.Mesh.Nodes'; % Transpose for [x, y, z] format
    tetrahedra = model.Geometry.Mesh.Elements';
    % nParticles = size(initialPositions, 1);

    % ************ Custom for the Jacobian (modify if needed) ************
    % We update the omesh elements
    % omesh.elements = tetrahedra;
    % Number of Agents
    N = 8;
    % Agent's Initial Positions (Each one at a vertex of the object)
    agent_poses = [ 0.9,  0.6,  0.6;... % Agent 1
                    0.9, -0.6,  0.6;... % Agent 2
                   -0.9, -0.6,  0.6;... % Agent 3
                   -0.9,  0.6,  0.6;... % Agent 4
                    0.9,  0.6,  0.0;... % Agent 5
                    0.9, -0.6,  0.0;... % Agent 6
                   -0.9, -0.6,  0.0;... % Agent 7
                   -0.9,  0.6,  0.0];   % Agent 8
    % Object's Initial Centroid Position (Mean of the Agent's positions for this problem)
    g0 = mean(agent_poses)';
    % Initial Scale Factor
    s0 = 1; % No scaling
    % We define the initial configuration
    p_init = reshape(agent_poses', [3*N, 1]);
    K = kron(eye(N) - (1/N)*ones(N,N), eye(3)); % centering matrix
    % Rest of params
    p00 = K * p_init;
    R0 = eul2rotm([0 0 0], 'XYZ');
    R0 = kron(eye(N), R0);
    p0 = (s0 * R0 * p00) + reshape(g0 * ones(1,N), [3*N,1]);
    % ARAP
    handled_nodes = [];
    for i = 0:N-1
        differences = omesh.shape - p_init(3*i+1:3*i+3)';       % Subtract target from each column
        dists = sqrt(sum(differences.^2, 2)); % Euclidean distances for each column
        [~, idx] = min(dists);          % minDist is the minimum distance, idx is the column index
        handled_nodes = [handled_nodes, idx];
    end
    n_iters_arap = 3; % choose a value of 2 or 3: the higher the value the stiffer the simulated object
    arap_params = create_params_for_arap(omesh, handled_nodes, [], n_iters_arap);  % create model parameters
    % ********************************************************************
    
    data = struct(...
        'positions', initialPositions,...
        'initialPositions', initialPositions,...
        'surfTriangles', surfTriangles,...
        'initialTetPoses', initialTetPositions,...
        'meshTetrahedra', tetrahedra,...
        'meshModel', model.Geometry.Mesh,...
        'fixed', false(nParticles, 1),...
        'selected', false(nParticles, 1),...
        'prevPartSelected', NaN,...
        'selectThreshold', selectionThreshold,...
        'poissonRatio', 0.3,...
        'YoungModulus', 210e10,...
        'scatterPlot', [],...
        'initialScatterPlot', [],...
        'ax', ax,...
        'shiftPressed', false,...
        'mousePressed', false,...
        'loadlabel', handles.loadLabel,...
        'handles', handles,...
        'recordJacobian', false,...
        'agentActions', [],...
        'particleDisplacements', [],...
        'vmValues', [],...
        'arapParams', arap_params,...
        'agentPoses', agent_poses,...
        'agentIndices', handled_nodes,...
        'dt', 1);
        % 'dt', 0.01);
    
    hold(ax, 'on');
    data.initialScatterPlot = scatter3(ax, data.initialPositions(:,1), data.initialPositions(:,2), data.initialPositions(:,3), ...
        'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.3);
    data.scatterPlot = scatter3(ax, data.positions(:,1), data.positions(:,2), data.positions(:,3), ...
        'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1.0);
    hold(ax, 'off');
    fig.UserData = data;
    updatePlot();
    
    % Set callbacks
    fig.WindowButtonDownFcn = @axesClicked;
    fig.WindowButtonUpFcn = @axesReleased;
    fig.WindowButtonMotionFcn = @mouseMove;
    fig.WindowKeyPressFcn = @keyPress;
    fig.WindowKeyReleaseFcn = @keyRelease;


    % +++ Callbacks +++

    % It updates the color and the positions of the particles based on the
    % figure information
    function updatePlot()
        data = fig.UserData;
        colors = repmat([0 0 1], size(data.positions,1), 1); % Blue base
        
        % Update colors based on state
        colors(data.fixed & ~data.selected,:) = repmat([0 0 0], sum(data.fixed & ~data.selected), 1); % Black
        colors(data.selected & ~data.fixed,:) = repmat([1 0 0], sum(data.selected & ~data.fixed), 1); % Red
        colors(data.selected & data.fixed,:) = repmat([1 0 1], sum(data.selected & data.fixed), 1); % Magenta
        
        data.scatterPlot.XData = data.positions(:,1);
        data.scatterPlot.YData = data.positions(:,2);
        data.scatterPlot.ZData = data.positions(:,3);
        data.scatterPlot.CData = colors;
        drawnow;
    end

    % It updates the von Mises plot
    function updateVonMises()
        data = fig.UserData;

        % We first get the interpolation of the tetrahedral mesh vertices
        surfaceDisplacements = data.positions - data.initialPositions;
        [currMeshPositions, currMeshDisp] = interpolateParticles(double(data.initialPositions), double(surfaceDisplacements), data.initialTetPoses);

        % With this we can compute the Von Mises Stress in 3D
        [VMSigma, Stress] = Compute3DvonMises(data.initialTetPoses, currMeshPositions, data.meshTetrahedra, data.YoungModulus, data.poissonRatio);

        pdeplot3D(data.meshModel,...
                  'Parent', data.ax_vm, ...  % Set custom axes
                  'ColorMapData', VMSigma, ...
                  'Deformation', currMeshDisp);
        
        % Save if we are recording values
        if data.recordJacobian
            data.vmValues = [data.vmValues, VMSigma];
        end
        fig.UserData = data;
    end

    % Toggle visibility function
    function toggleInitialPositions(src, ~)
        data = fig.UserData;
        if src.Value == "on"
            data.initialScatterPlot.Visible = 'on';
        else
            data.initialScatterPlot.Visible = 'off';
        end
        fig.UserData = data;
    end

    % Function for recording the inputs and the outputs measured in the
    % system for computing the Jacobian
    function recordJacobian(~,~)
        data = fig.UserData;

        if data.recordJacobian % If we were recording
            % We open a window to save the .mat file with the data
            [filename, pathname] = uiputfile('*.mat', 'Save MAT file as');
            if isequal(filename,0) || isequal(pathname,0)
                disp('User canceled');
            else
                fullFileName = fullfile(pathname, filename);
                % Save your variables into the file
                agent_actions = data.agentActions;
                particle_displacements = data.particleDisplacements;
                n_experiments = length(agent_actions);

                if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
                    mesh_model = data.meshModel;
                    vonMisesValues = data.vmValues;
                    save(fullFileName, "agent_actions", "particle_displacements", "n_experiments", "mesh_model", "vonMisesValues");
                else
                    save(fullFileName, "agent_actions", "particle_displacements", "n_experiments");
                end
            end

            % We reset everything to its inital state
            data.handles.recordJButton.Text = 'Record J';
            data.handles.recordJButton.BackgroundColor = [0.96 0.96 0.96];
            data.recordJacobian = false;
            data.agentActions = [];
            data.particleDisplacements = [];
            data.vmValues = [];
        else
            data.handles.recordJButton.Text = 'Stop Rec';
            data.handles.recordJButton.BackgroundColor = "red";
            data.recordJacobian = true;
        end

        fig.UserData = data;
    end

    % Load data callback
    function loadData(~, ~)
        % Open a file selection dialog for .h5 or .mat files
        [fileName, filePath] = uigetfile({'*.h5;*.mat', 'HDF5 or MAT files'}, 'Select a file');
        
        if fileName ~= 0
            
            % Load the data based on file extension
            if endsWith(fileName, '.mat')
                % Load the MAT file
                loadedData = load(fullfile(filePath, fileName));

            elseif endsWith(fileName, '.h5')
                % Load the HDF5 file
                hdf52mat(fullfile(filePath, fileName));
                loadedData = load(strcat(filepath,filename,".mat"));
            end

            if exist("loadedData","var")
                % Get the object name
                objectname = keys(loadedData.unity_keys.DefObj);
                % Get the first object map
                defObj = loadedData.Unity(string(objectname(1)));
    
                % -- You can change this to select the info that you want
                % to plot, right now we will select the vertex info but you
                % can select between ("particles_info", "vertex_info") --
    
                % Get the vertex info
                vertex_info = defObj("vertex_info");
                % We store only the initial position as we will deform it
                % interactively
                X = vertex_info(1).x;
                Y = vertex_info(1).y;
                Z = vertex_info(1).z;

                % Get the triangles
                triang_info = defObj("mesh_triangles");
                v1 = triang_info(1).x(:) + 1;
                v2 = triang_info(1).y(:) + 1;
                v3 = triang_info(1).z(:) + 1;

                surfTriangles = [v1(:), v2(:), v3(:)];

                % Then we generate the tetrahedral mesh
                [initialTetPositions, tetrahedra] = generateTetMesh(X(:), Y(:), Z(:), surfTriangles);
    
                initialPositions = [X(:), Y(:), Z(:)];
                nParticles = size(initialPositions, 1);
    
                % +++ Figure Handling +++
                % Store data in figure
                data.positions=initialPositions;
                data.initialPositions = initialPositions;
                data.initialTetPositions = initialTetPositions;
                data.meshTetrahedra = tetrahedra;
                data.fixed = false(nParticles, 1);
                data.selected = false(nParticles, 1);
                data.prevPartSelected = NaN;
                data.shiftPressed = false;
                data.mousePressed = false;
                data.loadlabel.Text = fileName;

                
                cla(data.ax); % Reset the plot
                % Update the plot data
                hold(data.ax,"on");
                data.initialScatterPlot = scatter3(data.ax, data.initialPositions(:,1), data.initialPositions(:,2), data.initialPositions(:,3), ...
                    'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.3); % Initial positions with transparency
                data.scatterPlot = scatter3(data.ax, data.positions(:,1), data.positions(:,2), data.positions(:,3), ...
                    'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1.0); % Current positions
                hold(data.ax,"off");
                fig.UserData = data;
                updatePlot();


            end
        end
    end

    function [particles, tetrahedra] = generateTetMesh(X, Y, Z, surfTriangles)
        data = fig.UserData;

        gm = fegeometry(double([X(:), Y(:), Z(:)]), surfTriangles);
        model = femodel(AnalysisType="structuralStatic", Geometry=gm);
        model.MaterialProperties = materialProperties(YoungsModulus=data.YoungModulus, PoissonsRatio=data.poissonRatio, MassDensity=2.7e-6);
        model = generateMesh(model, GeometricOrder="linear");
        particles = model.Geometry.Mesh.Nodes'; % Transpose for [x, y, z] format
        tetrahedra = model.Geometry.Mesh.Elements';    
        data.meshModel = model.Geometry.Mesh;
    end

    % Particle selection
    function axesClicked(~, ~)
        data = fig.UserData;
        % We keep track of the mouse pressing
        data.mousePressed = true;
        cursorPos = get(data.ax, 'CurrentPoint'); % Cursor in 3D

        % Find nearest particle
        P1 = cursorPos(1,:); % Start point of the line
        P2 = cursorPos(2,:); % End point of the line
        dir = P2 - P1;
        dirNorm = norm(dir);
        
        if dirNorm == 0
            % Handle case where line is a single point
            dists = vecnorm(data.positions - P1, 2, 2);
        else
            v = data.positions - P1;
            dirRep = repmat(dir, size(v, 1), 1); 
            crossProd = cross(v, dirRep, 2);
            dists = vecnorm(crossProd, 2, 2) / dirNorm;
        end
        [minDist, idx] = min(dists);

        % If within threshold, check depth to determine topmost particle
        if minDist < data.selectThreshold     
        % Get the data index from the clicked data point

            % We store the previous info of idx selection to toggle it
            idx_selected = data.selected(idx);
            data.prevPartSelected = idx;
            
            % Update selection
            if ~data.shiftPressed
                data.selected(:) = false;
            end
    
            data.selected(idx) = ~idx_selected;
            
            fig.UserData = data;
            updatePlot();
        end
    end


    function mouseMove(~, ~)
        data = fig.UserData;
        if data.mousePressed && data.shiftPressed
            cursorPos = get(data.ax, 'CurrentPoint'); % Cursor in 3D
            
            % Find nearest particle
            P1 = cursorPos(1,:); % Start point of the line
            P2 = cursorPos(2,:); % End point of the line
            dir = P2 - P1;
            dirNorm = norm(dir);
            
            if dirNorm == 0
                % Handle case where line is a single point
                dists = vecnorm(data.positions - P1, 2, 2);
            else
                v = data.positions - P1;
                dirRep = repmat(dir, size(v, 1), 1); 
                crossProd = cross(v, dirRep, 2);
                dists = vecnorm(crossProd, 2, 2) / dirNorm;
            end
            [minDist, idx] = min(dists);
    
            % If within threshold, check depth to determine topmost particle
            if minDist < data.selectThreshold
                if data.prevPartSelected ~= idx
                    data.selected(idx) = ~data.selected(idx);
                    data.prevPartSelected = idx;
                end
                fig.UserData = data;
                updatePlot();
            end
        end
    end

    function axesReleased(~, ~)
        data = fig.UserData;
        data.mousePressed = false;
        fig.UserData = data;
    end

    % Update the distance threshold
    function updateThreshold(~, ~)
        data = fig.UserData;
        data.selectThreshold = handles.thresholdEdit.Value;
        fig.UserData = data;
    end

    % Update the poisson ratio
    function updatePoisson(~, ~)
        data = fig.UserData;
        data.poissonRatio = handles.poissonEdit.Value;
        fig.UserData = data;
    end

    % Update the Young Modulus
    function updateYoungModulus(~, ~)
        data = fig.UserData;
        data.YoungModulus = handles.youngEdit.Value;
        fig.UserData = data;
    end

    function toggleFixed(~, ~)
        data = fig.UserData;
        % data.fixed(data.selected) = src.Value;
        allSame = isscalar(unique(data.fixed(data.selected)));
        if allSame
            data.fixed(data.selected) = ~data.fixed(data.selected);
        else
            data.fixed(data.selected) = true;
        end
        fig.UserData = data;
        updatePlot();
    end

    % Apply the force to the particles
    function applyForce(~, ~)
        % --------- Old --------- 
        % data = fig.UserData;
        % force = [handles.forceXEdit.Value, handles.forceYEdit.Value, handles.forceZEdit.Value];
        % 
        % grabbing_points = data.positions(data.selected & ~data.fixed,:);
        % fixed_points = data.positions(data.fixed,:);
        % 
        % data.positions = stretchFromPoints(data.positions, grabbing_points, fixed_points, data.poissonRatio, force);
        % 
        % fig.UserData = data;
        % updatePlot();
        % 
        % if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
        %     updateVonMises();
        % end
        % ----------------------- 



        % --------- New (ARAP) --------- 
        data = fig.UserData;
        force = [handles.forceXEdit.Value, handles.forceYEdit.Value, handles.forceZEdit.Value];

        grabbing_points = data.positions(data.selected & ~data.fixed,:);
        fixed_points = data.positions(data.fixed,:);

        differences = data.agentPoses - grabbing_points;       % Subtract target from each column
        dists = sqrt(sum(differences.^2, 2)); % Euclidean distances for each column
        [~, idx] = min(dists);          % minDist is the minimum distance, idx is the column index

        data.agentPoses(idx,:) = data.agentPoses(idx,:) + force * data.dt;

        prev_positions = data.positions;

        data.positions = simulate_object_arap(arap_params, data.positions', data.agentPoses')';

        % If we are recording we save the data
        if data.recordJacobian % If we were recording
            % Agent Actions
            actions = zeros(size(data.agentPoses,1)*3,1);
            actions(3*(idx-1)+1:3*(idx-1)+3) = force;
            data.agentActions = [data.agentActions;actions'];
    
            % Particle Displacements
            data.particleDisplacements = cat(3,data.particleDisplacements,data.positions-prev_positions);
        end

        fig.UserData = data;
        updatePlot();

        if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
            updateVonMises();
        end
    end

    % Move the whole object
    function moveObject(~, ~)
        data = fig.UserData;
        move = [handles.moveXEdit.Value, handles.moveYEdit.Value, handles.moveZEdit.Value];
        data.positions = data.positions + move;
        fig.UserData = data;
        updatePlot();

        if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
            updateVonMises();
        end
    end

    % Reset the particles position to its initial
    function resetObject(~, ~)
        data = fig.UserData;
        data.positions = data.initialPositions;
        data.selected(:) = false;
        data.fixed(:) = false;
        data.agentPoses = [ 0.9,  0.6,  0.6;... % Agent 1
                            0.9, -0.6,  0.6;... % Agent 2
                           -0.9, -0.6,  0.6;... % Agent 3
                           -0.9,  0.6,  0.6;... % Agent 4
                            0.9,  0.6,  0.0;... % Agent 5
                            0.9, -0.6,  0.0;... % Agent 6
                           -0.9, -0.6,  0.0;... % Agent 7
                           -0.9,  0.6,  0.0];   % Agent 8
        fig.UserData = data;
        updatePlot();

        if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
            updateVonMises();
        end
    end

    % Group selection (this means that while shift is pressed down you can
    % select multiple particles)
    function keyPress(~, event)
        data = fig.UserData;
        if strcmp(event.Key, 'shift')
            data.shiftPressed = true;
            fig.UserData = data;
        end
    end

    % Group selection (when you release shift the multiple selection is
    % deactivated)
    function keyRelease(~, event)
        data = fig.UserData;
        if strcmp(event.Key, 'shift')
            data.shiftPressed = false;
            fig.UserData = data;
        end
    end

    % +++ Von Mises Slider Callback +++
    function toggleVonMises(src, ~)
        data = fig.UserData;
        if strcmp(src.Value, "on")
            % Expand figure width
            fig.Position = [100 100 1200 600];
            
            % Reposition original axes to the right
            data.ax.Position = [650, 200, 500, 400];
            
            % Create new von Mises axes
            data.ax_vm = uiaxes(fig, 'Position', [50 200 500 400], 'Box', 'on', 'GridLineStyle', '-');
            data.ax_vm.XGrid = 'on';
            data.ax_vm.YGrid = 'on';
            data.ax_vm.ZGrid = 'on';
            xlabel(data.ax_vm, 'X');
            ylabel(data.ax_vm, 'Y');
            zlabel(data.ax_vm, 'Z');
            
            % Reposition UI components
            h = data.handles;
            offset = 400; % Expand by 400 pixels
            
            % Right-side components
            h.loadButton.Position(1) = 590 + offset;
            h.resetButton.Position(1) = 590 + offset;
            h.recordJButton.Position(1) = 590 + offset;
            h.vmSwitch.Position(1) = 720 + offset;
            h.initialSwitch.Position(1) = 720 + offset;
            h.loadLabel.Position(1) = 590 + offset;
            h.initialLabel.Position(1) = 690 + offset;
            h.vmLabel.Position(1) = 690 + offset;
            
            h.line3.Position(1) = 580 + offset;
            h.line4.Position(1) = 680 + offset;
            
            % Center-right components (Young, Poisson)
            centerOffset = 200;
            h.youngEdit.Position(1) = 460 + centerOffset;
            h.poissonEdit.Position(1) = 460 + centerOffset;
            h.youngLabel.Position(1) = 460 + centerOffset;
            h.poissonLabel.Position(1) = 460 + centerOffset;
            h.line2.Position(1) = 450 + centerOffset;
            
            % Center-left components (Force, Move)
            h.forceXEdit.Position(1) = 150 + centerOffset;
            h.forceYEdit.Position(1) = 220 + centerOffset;
            h.forceZEdit.Position(1) = 290 + centerOffset;
            h.applyForceButton.Position(1) = 360 + centerOffset;
            h.forceXLabel.Position(1) = 150 + centerOffset;
            h.forceYLabel.Position(1) = 220 + centerOffset;
            h.forceZLabel.Position(1) = 290 + centerOffset;
            
            h.moveXEdit.Position(1) = 150 + centerOffset;
            h.moveYEdit.Position(1) = 220 + centerOffset;
            h.moveZEdit.Position(1) = 290 + centerOffset;
            h.moveButton.Position(1) = 360 + centerOffset;
            h.moveXLabel.Position(1) = 150 + centerOffset;
            h.moveYLabel.Position(1) = 220 + centerOffset;
            h.moveZLabel.Position(1) = 290 + centerOffset;
            
            h.line1.Position(1) = 140 + centerOffset;
            
            % Left-side components remain
            h.fixButton.Position(1) = 25;
            h.thresholdEdit.Position(1) = 20;
            h.thresholdLabel.Position(1) = 20;
            
        else
            % Revert figure size and axes
            fig.Position = [100 100 800 600];
            data.ax.Position = [50 200 700 400];
            if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
                delete(data.ax_vm);
                data = rmfield(data, 'ax_vm');
            end
            
            % Reset UI components to original positions
            h = data.handles;
            h.loadButton.Position(1) = 590;
            h.resetButton.Position(1) = 590;
            h.recordJButton.Position(1) = 590;
            h.vmSwitch.Position(1) = 720;
            h.initialSwitch.Position(1) = 720;
            h.loadLabel.Position(1) = 590;
            h.initialLabel.Position(1) = 690;
            h.vmLabel.Position(1) = 690;
            
            h.line3.Position(1) = 580;
            h.line4.Position(1) = 680;
            
            h.youngEdit.Position(1) = 460;
            h.poissonEdit.Position(1) = 460;
            h.youngLabel.Position(1) = 460;
            h.poissonLabel.Position(1) = 460;
            h.line2.Position(1) = 450;
            
            h.forceXEdit.Position(1) = 150;
            h.forceYEdit.Position(1) = 220;
            h.forceZEdit.Position(1) = 290;
            h.applyForceButton.Position(1) = 360;
            h.forceXLabel.Position(1) = 150;
            h.forceYLabel.Position(1) = 220;
            h.forceZLabel.Position(1) = 290;
            
            h.moveXEdit.Position(1) = 150;
            h.moveYEdit.Position(1) = 220;
            h.moveZEdit.Position(1) = 290;
            h.moveButton.Position(1) = 360;
            h.moveXLabel.Position(1) = 150;
            h.moveYLabel.Position(1) = 220;
            h.moveZLabel.Position(1) = 290;
            
            h.line1.Position(1) = 140;
            
            h.fixButton.Position(1) = 25;
            h.thresholdEdit.Position(1) = 20;
            h.thresholdLabel.Position(1) = 20;
        end
        fig.UserData = data;
    end

% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% +++++++++++++++++++++ Additional deformation methods ++++++++++++++++++++
% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   function [deformed_vertices] = stretchFromPoint(original_vertices, grabbing_point, fixed_points, poisson_ratio, stretch_amount)
        % Ensure stretch_amount is a row vector
        stretch_amount = stretch_amount(:)';
        
        % Calculate force magnitude and direction
        s = norm(stretch_amount);
        if s == 0
            deformed_vertices = original_vertices;
            return;
        end
        d = stretch_amount / s;
        
        % Get orthonormal basis vectors u and v perpendicular to d
        [u, v] = getOrthonormalBasis(d);
        
        % Compute vectors from grabbing point
        vec = original_vertices - grabbing_point;
        
        % Project onto u and v directions
        component_u = vec * u';
        component_v = vec * v';
        
        % Compute max components for normalization
        max_component_u = max(abs(component_u));
        max_component_v = max(abs(component_v));
        
        % Compute weights based on distance to grabbing point and fixed points
        distance_to_grab = sqrt(sum((original_vertices - grabbing_point).^2, 2));
        max_distance_to_grab = max(distance_to_grab);
        weights = 1 - (distance_to_grab / (max_distance_to_grab + eps));
        
        % Compute minimum distance to any fixed point
        min_distance_to_fixed = zeros(size(original_vertices, 1), 1);
        for i = 1:size(fixed_points, 1)
            dist = sqrt(sum((original_vertices - fixed_points(i, :)).^2, 2));
            if i == 1
                min_distance_to_fixed = dist;
            else
                min_distance_to_fixed = min(min_distance_to_fixed, dist);
            end
        end
        max_min_distance = max(min_distance_to_fixed);
        fixed_weights = min_distance_to_fixed / (max_min_distance + eps);
        weights = weights .* fixed_weights;
        
        % Calculate axial displacement
        axial_displacement = (s * weights) .* d;
        
        % Calculate displacements in u and v directions due to Poisson effect
        scaled_component_u = component_u / (max_component_u + eps);
        scaled_component_v = component_v / (max_component_v + eps);
        
        displacement_u = (-poisson_ratio * s) * weights .* scaled_component_u .* u;
        displacement_v = (-poisson_ratio * s) * weights .* scaled_component_v .* v;
        
        % Combine all displacements
        total_displacement = axial_displacement + displacement_u + displacement_v;
        
        % Apply displacement to original vertices
        deformed_vertices = original_vertices + total_displacement;
    end
    
    function [u, v] = getOrthonormalBasis(d)
        d = d / norm(d);
        % Choose a vector not parallel to d
        if abs(dot(d, [1, 0, 0])) < 1 - 1e-6
            temp = [1, 0, 0];
        else
            temp = [0, 1, 0];
        end
        u = cross(d, temp);
        u = u / norm(u);
        v = cross(d, u);
        v = v / norm(v);
    end

    function [deformed_vertices] = stretchFromPoints(original_vertices, grabbing_points, fixed_points, poisson_ratio, stretch_amount)
        if find(grabbing_points == 1)
            % Ensure stretch_amount is a row vector
            stretch_amount = stretch_amount(:)';
            
            % Compute center of mass of grabbing points
            grabbing_center = mean(grabbing_points, 1);
            
            % Compute displacement for grabbing points (they all move together)
            grabbing_displacement = repmat(stretch_amount, size(grabbing_points, 1), 1);
            
            % Compute distance of each vertex to the closest grabbing point
            distance_to_grab = min(vecnorm(original_vertices - permute(grabbing_points, [3, 2, 1]), 2, 2), [], 3);
            max_distance_to_grab = max(distance_to_grab) + eps;
            
            % Compute distance of each vertex to the closest fixed point
            if isempty(fixed_points)
                min_distance_to_fixed = ones(size(original_vertices, 1), 1);
            else
                 min_distance_to_fixed = min(vecnorm(original_vertices - permute(fixed_points, [3, 2, 1]), 2, 2), [], 3);
            end
            max_min_distance = max(min_distance_to_fixed) + eps;
            
            % Compute deformation weights: 
            % - Close to grabbing points → Move almost like them
            % - Close to fixed points → Move very little
            % - In between → Smooth interpolation
            grab_weight = 1 - (distance_to_grab / max_distance_to_grab);
            fixed_weight = min_distance_to_fixed / max_min_distance;
            weights = grab_weight .* fixed_weight;
            
            % Compute overall displacement field
            total_displacement = weights .* stretch_amount;
            
            % Apply Poisson contraction perpendicular to stretch direction
            s = norm(stretch_amount);
            d = stretch_amount / s;
            [u, v] = getOrthonormalBasis(d);
            
            % Compute projections for contraction
            vec = original_vertices - grabbing_center;
            component_u = vec * u';
            component_v = vec * v';
            
            max_component_u = max(abs(component_u)) + eps;
            max_component_v = max(abs(component_v)) + eps;
            
            scaled_component_u = component_u / max_component_u;
            scaled_component_v = component_v / max_component_v;
            
            contraction_u = (-poisson_ratio * s) * weights .* scaled_component_u .* u;
            contraction_v = (-poisson_ratio * s) * weights .* scaled_component_v .* v;
            
            % Combine all displacements
            total_displacement = total_displacement + contraction_u + contraction_v;
            
            % Apply displacement
            deformed_vertices = original_vertices + total_displacement;
            
            % Ensure grabbing points move rigidly (override their deformation)
            for i = 1:size(grabbing_points, 1)
                idx = ismember(original_vertices, grabbing_points(i, :), 'rows');
                deformed_vertices(idx, :) = grabbing_points(i, :) + stretch_amount;
            end
        else
            deformed_vertices = original_vertices;
        end
    end

    % Interpolation for getting the tetrahedral mesh positions based on the
    % surface positions
    function [tetPoses, tetMeshDisplacements] = interpolateParticles(surfaceInitialPose, surfaceDisplacements, tetMeshInitPose)
        % We generate the interpolation function for the surface particles
        interpX = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,1), 'linear', 'nearest');
        interpY = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,2), 'linear', 'nearest');
        interpZ = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,3), 'linear', 'nearest');
        
        % We get the displacement interpolations of the mesh nodes based on
        % the previous interpolation function
        tetMeshDisplacements = zeros(size(tetMeshInitPose, 1), 3);
        tetMeshDisplacements(:,1) = interpX(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));
        tetMeshDisplacements(:,2) = interpY(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));
        tetMeshDisplacements(:,3) = interpZ(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));

        % We finally get our interpolated mesh poses
        tetPoses = tetMeshInitPose + tetMeshDisplacements;
    end

% ----------------------------------------------------------------------------------------------------------------------------------------------------------------
% -------------------------------------- Von Mises ---------------------------------------------------------------------------------------------------------------
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    function [tet_VMSigma, tet_Stress] = Compute3DvonMises(VertexInitPoses, VertexFinalPoses, MeshTetrahedrons, E, nu)
        % von Mises With our data
        n_nodes = length(VertexInitPoses);
        % Initialize displacement vector (12x1xN_tet)
        u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));
    
        % Compute element stiffness matrix (6x6xN_tet)
        Ce = Ce3DMatrixComputation(E, nu);
        
        % Compute strain-displacement matrices (6x12xN_tet)
        Le = Compute3DBe(MeshTetrahedrons, VertexInitPoses);
        
        % Construct displacement vectors for all tetrahedrons
        for elem = 1:size(MeshTetrahedrons, 1)
            nodes = MeshTetrahedrons(elem, :); % Get the global node indices
            for node = 1:4
                u_hat_e(3*node-2,1,elem) = VertexFinalPoses(nodes(node),1) - VertexInitPoses(nodes(node),1); % x-displacement
                u_hat_e(3*node-1,1,elem) = VertexFinalPoses(nodes(node),2) - VertexInitPoses(nodes(node),2); % y-displacement
                u_hat_e(3*node,1,elem)   = VertexFinalPoses(nodes(node),3) - VertexInitPoses(nodes(node),3); % z-displacement
            end
        end
        
        
        % Compute strains (6x1xN_tet)
        strains_disp = pagemtimes(Le, u_hat_e);
        
        % Compute stresses (6x1xN_tet)
        stresses_disp = pagemtimes(Ce, strains_disp);
        
        % Initialize node-based stress tensor
        tet_Stress = zeros(6,1,n_nodes);
        node_contributions = zeros(n_nodes, 1); % Track contributions per node
        
        % Accumulate stresses for each node
        for elem = 1:size(MeshTetrahedrons, 1)
            nodes = MeshTetrahedrons(elem, :);
            stress_elem = stresses_disp(:,:,elem);
            for i = 1:4
                tet_Stress(:,:,nodes(i)) = tet_Stress(:,:,nodes(i)) + stress_elem;
                node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
            end
        end
        
        % Average stress at each node
        for node = 1:n_nodes
            if node_contributions(node) > 0
                tet_Stress(:,:,node) = tet_Stress(:,:,node) / node_contributions(node);
            end
        end
        
        % Compute von Mises stress at each node
        tet_VMSigma = zeros(n_nodes, 1);
        for node = 1:n_nodes
            sigma_12 = tet_Stress(1,:,node) - tet_Stress(2,:,node); % sigma_x - sigma_y
            sigma_23 = tet_Stress(2,:,node) - tet_Stress(3,:,node); % sigma_y - sigma_z
            sigma_31 = tet_Stress(3,:,node) - tet_Stress(1,:,node); % sigma_z - sigma_x
            tet_VMSigma(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
                3 * (tet_Stress(4,:,node)^2 + tet_Stress(5,:,node)^2 + tet_Stress(6,:,node)^2));
        end
    end
    
    % Computation of the Elasticity Matrix (Material Stifness Matrix) from
    % Hooke's law for 3D isotropic materials
    function [Ce] = Ce3DMatrixComputation(E, nu)
    
        poissonMatrix = [1-nu nu nu 0 0 0;...
                          nu 1-nu nu 0 0 0;...
                          nu nu 1-nu 0 0 0;...
                          0 0 0 (1-2*nu)/2 0 0;...
                          0 0 0 0 (1-2*nu)/2 0;...
                          0 0 0 0 0 (1-2*nu)/2];
    
        Ce = E/((1+nu)*(1-2*nu)) * poissonMatrix;
    
    end
    
    
    function B = Compute3DBe(MeshTetraedrons, VertexPose)
        N_tet = length(MeshTetraedrons);
        B = zeros(6,12,N_tet);
        for e = 1:N_tet
            % Extract indices of the four vertices for the current tetrahedron
            indices = MeshTetraedrons(e, :);
            P1 = VertexPose(indices(1), :); % First point
            P2 = VertexPose(indices(2), :); % Second point
            P3 = VertexPose(indices(3), :); % Third point
            P4 = VertexPose(indices(4), :); % Fourth point
    
            grads = compute3DShapeFunctionGradients(P1, P2, P3, P4);
            % V = abs(det([1, P1; 1, P2; 1, P3; 1, P4])) / 6;
    
            for i = 1:4
                % Each column block corresponds to one vertex
                B(1, 3*i-2, e) = grads(i ,1); % dN_i/dx
                B(2, 3*i-1, e) = grads(i, 2); % dN_i/dy
                B(3, 3*i, e)   = grads(i, 3); % dN_i/dz
                B(4, 3*i-2, e) = grads(i, 2); % dN_i/dy
                B(4, 3*i-1, e) = grads(i, 1); % dN_i/dx
                B(5, 3*i-1, e) = grads(i, 3); % dN_i/dz
                B(5, 3*i, e)   = grads(i, 2); % dN_i/dy
                B(6, 3*i-2, e) = grads(i, 3); % dN_i/dz
                B(6, 3*i, e)   = grads(i, 1); % dN_i/dx
            end
        end
    end
    
    function grads = compute3DShapeFunctionGradients(P1, P2, P3, P4)
        % Computes the gradients of the shape functions for a tetrahedron
        % Inputs:
        % P1, P2, P3, P4: Coordinates of the tetrahedron vertices as 1x3 arrays
        % Output:
        % grads: 3x4 matrix of shape function gradients
    
        % Build the Jacobian matrix (3x3 matrix)
        J = [P2 - P1;  % Vector P1 -> P2
             P3 - P1;  % Vector P1 -> P3
             P4 - P1]'; % Vector P1 -> P4
    
        % Compute the determinant of the Jacobian to ensure it is non-zero
        detJ = det(J);
        if abs(detJ) < 1e-12
            error('Jacobian is singular or nearly singular. Check input points.');
        end
    
        % Compute the inverse of the Jacobian matrix
        invJ = inv(J);
    
        % Gradients of shape functions in local coordinates (reference tetrahedron)
        % Reference shape function gradients with respect to local coords (ξ, η, ζ)
        local_grads = [-1, -1, -1;  % ∂N1/∂ξ, ∂N1/∂η, ∂N1/∂ζ
                        1,  0,  0;  % ∂N2/∂ξ, ∂N2/∂η, ∂N2/∂ζ
                        0,  1,  0;  % ∂N3/∂ξ, ∂N3/∂η, ∂N3/∂ζ
                        0,  0,  1]; % ∂N4/∂ξ, ∂N4/∂η, ∂N4/∂ζ (4x3)
    
        % Transform local gradients to global coordinates using the Jacobian inverse
        grads = (invJ' * local_grads')'; % 3x4 matrix
    end
end