function ProgramableDeformableObjectGUI
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
    handles.youngEdit = uieditfield(fig, 'numeric', 'Position', [460 140 110 22], 'Value', 3000, 'ValueChangedFcn', @updateYoungModulus);
    
    % Poisson Ratio
    handles.poissonEdit = uieditfield(fig, 'numeric', 'Position', [460 80 110 22], 'Value', 0.01, 'ValueChangedFcn', @updatePoisson);
    
    % Load Data Button
    handles.loadButton = uibutton(fig, 'Position', [590 160 80 22], 'Text', 'Load Data', 'ButtonPushedFcn', @loadData);
    
    % Initial Positions Slider
    handles.initialSwitch = uiswitch(fig, 'slider', 'Items', ["off","on"], 'Value', "on", 'Position', [720 80 40 20], 'ValueChangedFcn', @toggleInitialPositions);
    
    % Von Mises Stress Slider
    handles.vmSwitch = uiswitch(fig, 'slider', 'Items', ["off", "on"], 'Value', "off", 'Position', [720, 140, 40, 20], 'ValueChangedFcn', @toggleVonMises);
    
    % Record Jacobian Button
    handles.recordJButton = uibutton(fig, 'Position', [590 115 80 22], 'Text', 'Record J', 'ButtonPushedFcn', @recordJacobian);

    % Apply Deformation
    handles.applyDeformations = uibutton(fig, 'Position', [360 40 80 22], 'Text', 'Apply Deformation', 'ButtonPushedFcn', @applyDeformationsProgrammatically);
    handles.deformationEdit = uieditfield(fig, 'text', 'Position', [150 40 200 22], 'Value', '[]');

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
    nx = 5; % number of nodes in x direction of the grid
    ny = 5; % number of nodes in y direction of the grid
    nz = 5; % number of nodes in z direction of the grid
    
    sx = 1; % length of the object in x direction
    sy = 1; % length of the object in y direction
    sz = 1; % length of the object in z direction
    
    scale_ob = 1;
    omesh = create_mesh3D(nx, ny, nz, sx * scale_ob, sy * scale_ob, sz * scale_ob);
    omesh.shape = omesh.shape + [0, 0, -1*(min(omesh.shape(:,3))<0)*min(omesh.shape(:,3))];
    X = omesh.shape(:,1);
    Y = omesh.shape(:,2);
    Z = omesh.shape(:,3);

    initialPositions = [X(:), Y(:), Z(:)];
    nParticles = size(initialPositions, 1);
    selectionThreshold = 0.2;

    % Get the triangles and tetrahedra of the object
    shp = alphaShape(X(:), Y(:), Z(:));
    surfTriangles = boundaryFacets(shp);
    initialTetPositions = omesh.shape;
    tetrahedra = omesh.elements;

    % Jacobian setup
    N = 8;
    agent_poses = [ 0.5,  0.5,  1.0;...
                    0.5, -0.5,  1.0;...
                   -0.5, -0.5,  1.0;...
                   -0.5,  0.5,  1.0;...
                    0.5,  0.5,  0.0;...
                    0.5, -0.5,  0.0;...
                   -0.5, -0.5,  0.0;...
                   -0.5,  0.5,  0.0];
    g0 = mean(agent_poses)';
    s0 = 1;
    p_init = reshape(agent_poses', [3*N, 1]);
    K = kron(eye(N) - (1/N)*ones(N,N), eye(3));
    p00 = K * p_init;
    R0 = eul2rotm([0 0 0], 'XYZ');
    R0 = kron(eye(N), R0);
    p0 = (s0 * R0 * p00) + reshape(g0 * ones(1,N), [3*N,1]);
    handled_nodes = [];
    for i = 0:N-1
        differences = omesh.shape - p_init(3*i+1:3*i+3)';
        dists = sqrt(sum(differences.^2, 2));
        [~, idx] = min(dists);
        handled_nodes = [handled_nodes, idx];
    end
    n_iters_arap = 3;
    arap_params = create_params_for_arap(omesh, handled_nodes, [], n_iters_arap);

    data = struct(...
        'positions', initialPositions,...
        'initialPositions', initialPositions,...
        'surfTriangles', surfTriangles,...
        'initialTetPoses', initialTetPositions,...
        'meshTetrahedra', tetrahedra,...
        'meshModel', omesh,...
        'fixed', false(nParticles, 1),...
        'selected', false(nParticles, 1),...
        'prevPartSelected', NaN,...
        'selectThreshold', selectionThreshold,...
        'poissonRatio', 0.01,...
        'YoungModulus', 3000,...
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

    % New function to apply deformations programmatically
    function applyDeformationsProgrammatically(~, ~)
        forces = str2num(handles.deformationEdit.Value);
        data = fig.UserData;
        
        % Define deformation directions
        directions = {...
            [1, 0, 0], ... % X
            [0, 1, 0], ... % Y
            [0, 0, 1], ... % Z
            [1, 1, 0], ... % XY
            [1, 0, 1], ... % XZ
            [0, 1, 1], ... % YZ
            [1, 1, 1]  ... % XYZ
        };
        
        % Normalize directions
        for i = 1:length(directions)
            directions{i} = directions{i} / norm(directions{i});
        end
        
        % Start recording
        data.recordJacobian = true;
        data.agentActions = [];
        data.particleDisplacements = [];
        data.vmValues = [];
        
        % Iterate through each handled node
        for nodeIdx = data.agentIndices       
            disp(strcat("--> Analyzing agent : ", num2str(nodeIdx)));
            % Iterate through each force magnitude
            for forceMag = forces
                disp(strcat("     --> Applying force: ",num2str(forceMag)));
                % Iterate through each direction
                for dir = directions
                    % Reset object to initial state
                    resetObject([], []);

                    data = fig.UserData;
            
                    % Select the current node
                    data.selected(:) = false;
                    data.selected(nodeIdx) = true;

                    % Apply force in the current direction
                    force = forceMag * dir{1};
                    
                    % Update agent position
                    grabbing_points = data.positions(data.selected & ~data.fixed,:);
                    differences = data.agentPoses - grabbing_points;
                    dists = sqrt(sum(differences.^2, 2));
                    [~, idx] = min(dists);
                    
                    data.agentPoses(idx,:) = data.agentPoses(idx,:) + force * data.dt;
                    
                    % Store previous positions for displacement calculation
                    prev_positions = data.positions;
                    
                    % Simulate deformation using ARAP
                    data.positions = simulate_object_arap(data.arapParams, data.positions', data.agentPoses')';
                    
                    % Record data
                    actions = zeros(size(data.agentPoses,1)*3,1);
                    actions(3*(idx-1)+1:3*(idx-1)+3) = force;
                    data.agentActions = [data.agentActions; actions'];
                    data.particleDisplacements = cat(3, data.particleDisplacements, data.positions - prev_positions);
                    
                    % Update plot
                    fig.UserData = data;
                    updatePlot();

                    % Update von Mises if enabled
                    if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
                        updateVonMises();
                    end
                end
            end
        end
        
        % Save recorded data
        [filename, pathname] = uiputfile('*.mat', 'Save Deformation Data');
        if ~isequal(filename, 0) && ~isequal(pathname, 0)
            fullFileName = fullfile(pathname, filename);
            agent_actions = data.agentActions;
            particle_displacements = data.particleDisplacements;
            n_experiments = size(agent_actions, 1);
            mesh_model = data.meshModel;
            vonMisesValues = data.vmValues;
            save(fullFileName, "agent_actions", "particle_displacements", "n_experiments", "mesh_model", "vonMisesValues");
        end
        
        % Stop recording
        data.recordJacobian = false;
        data.handles.recordJButton.Text = 'Record J';
        data.handles.recordJButton.BackgroundColor = [0.96 0.96 0.96];
        fig.UserData = data;
    end

    % +++ Callbacks +++
    function updatePlot()
        data = fig.UserData;
        colors = repmat([0 0 1], size(data.positions,1), 1);
        colors(data.fixed & ~data.selected,:) = repmat([0 0 0], sum(data.fixed & ~data.selected), 1);
        colors(data.selected & ~data.fixed,:) = repmat([1 0 0], sum(data.selected & ~data.fixed), 1);
        colors(data.selected & data.fixed,:) = repmat([1 0 1], sum(data.selected & data.fixed), 1);
        data.scatterPlot.XData = data.positions(:,1);
        data.scatterPlot.YData = data.positions(:,2);
        data.scatterPlot.ZData = data.positions(:,3);
        data.scatterPlot.CData = colors;
        drawnow;
    end

    function updateVonMises()
        data = fig.UserData;
        surfaceDisplacements = data.positions - data.initialPositions;
        [currMeshPositions, currMeshDisp] = interpolateParticles(double(data.initialPositions), double(surfaceDisplacements), data.initialTetPoses);
        [VMSigma, Stress] = Compute3DvonMises(data.initialTetPoses, currMeshPositions, data.meshTetrahedra, data.YoungModulus, data.poissonRatio);
        % pdeplot3D(data.meshModel,...
        %           'Parent', data.ax_vm, ...
        %           'ColorMapData', VMSigma, ...
        %           'Deformation', currMeshDisp);
        trisurf(omesh.elements, data.initialPositions(:,1), data.initialPositions(:,2), data.initialPositions(:,3), VMSigma, 'EdgeColor', 'none', 'Parent', data.ax_vm);
        if data.recordJacobian
            data.vmValues = [data.vmValues, VMSigma];
        end
        fig.UserData = data;
    end

    function toggleInitialPositions(src, ~)
        data = fig.UserData;
        if src.Value == "on"
            data.initialScatterPlot.Visible = 'on';
        else
            data.initialScatterPlot.Visible = 'off';
        end
        fig.UserData = data;
    end

    function recordJacobian(~,~)
        data = fig.UserData;
        if data.recordJacobian
            [filename, pathname] = uiputfile('*.mat', 'Save MAT file as');
            if isequal(filename,0) || isequal(pathname,0)
                disp('User canceled');
            else
                fullFileName = fullfile(pathname, filename);
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

    function loadData(~, ~)
        [fileName, filePath] = uigetfile({'*.h5;*.mat', 'HDF5 or MAT files'}, 'Select a file');
        if fileName ~= 0
            if endsWith(fileName, '.mat')
                loadedData = load(fullfile(filePath, fileName));
            elseif endsWith(fileName, '.h5')
                hdf52mat(fullfile(filePath, fileName));
                loadedData = load(strcat(filePath, fileName, ".mat"));
            end
            if exist("loadedData","var")
                objectname = keys(loadedData.unity_keys.DefObj);
                defObj = loadedData.Unity(string(objectname(1)));
                vertex_info = defObj("vertex_info");
                X = vertex_info(1).x;
                Y = vertex_info(1).y;
                Z = vertex_info(1).z;
                triang_info = defObj("mesh_triangles");
                v1 = triang_info(1).x(:) + 1;
                v2 = triang_info(1).y(:) + 1;
                v3 = triang_info(1).z(:) + 1;
                surfTriangles = [v1(:), v2(:), v3(:)];
                [initialTetPositions, tetrahedra] = generateTetMesh(X(:), Y(:), Z(:), surfTriangles);
                initialPositions = [X(:), Y(:), Z(:)];
                nParticles = size(initialPositions, 1);
                data.positions = initialPositions;
                data.initialPositions = initialPositions;
                data.initialTetPositions = initialTetPositions;
                data.meshTetrahedra = tetrahedra;
                data.fixed = false(nParticles, 1);
                data.selected = false(nParticles, 1);
                data.prevPartSelected = NaN;
                data.shiftPressed = false;
                data.mousePressed = false;
                data.loadlabel.Text = fileName;
                cla(data.ax);
                hold(data.ax,"on");
                data.initialScatterPlot = scatter3(data.ax, data.initialPositions(:,1), data.initialPositions(:,2), data.initialPositions(:,3), ...
                    'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.3);
                data.scatterPlot = scatter3(data.ax, data.positions(:,1), data.positions(:,2), data.positions(:,3), ...
                    'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1);
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
        particles = model.Geometry.Mesh.Nodes';
        tetrahedra = model.Geometry.Mesh.Elements';
        data.meshModel = model.Geometry.Mesh;
    end

    function axesClicked(~, ~)
        data = fig.UserData;
        data.mousePressed = true;
        cursorPos = get(data.ax, 'CurrentPoint');
        P1 = cursorPos(1,:);
        P2 = cursorPos(2,:);
        dir = P2 - P1;
        dirNorm = norm(dir);
        if dirNorm == 0
            dists = vecnorm(data.positions - P1, 2, 2);
        else
            v = data.positions - P1;
            dirRep = repmat(dir, size(v, 1), 1);
            crossProd = cross(v, dirRep, 2);
            dists = vecnorm(crossProd, 2, 2) / dirNorm;
        end
        [minDist, idx] = min(dists);
        if minDist < data.selectThreshold
            idx_selected = data.selected(idx);
            data.prevPartSelected = idx;
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
            cursorPos = get(data.ax, 'CurrentPoint');
            P1 = cursorPos(1,:);
            P2 = cursorPos(2,:);
            dir = P2 - P1;
            dirNorm = norm(dir);
            if dirNorm == 0
                dists = vecnorm(data.positions - P1, 2, 2);
            else
                v = data.positions - P1;
                dirRep = repmat(dir, size(v, 1), 1);
                crossProd = cross(v, dirRep, 2);
                dists = vecnorm(crossProd, 2, 2) / dirNorm;
            end
            [minDist, idx] = min(dists);
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

    function updateThreshold(~, ~)
        data = fig.UserData;
        data.selectThreshold = handles.thresholdEdit.Value;
        fig.UserData = data;
    end

    function updatePoisson(~, ~)
        data = fig.UserData;
        data.poissonRatio = handles.poissonEdit.Value;
        fig.UserData = data;
    end

    function updateYoungModulus(~, ~)
        data = fig.UserData;
        data.YoungModulus = handles.youngEdit.Value;
        fig.UserData = data;
    end

    function toggleFixed(~, ~)
        data = fig.UserData;
        allSame = isscalar(unique(data.fixed(data.selected)));
        if allSame
            data.fixed(data.selected) = ~data.fixed(data.selected);
        else
            data.fixed(data.selected) = true;
        end
        fig.UserData = data;
        updatePlot();
    end

    function applyForce(~, ~)
        data = fig.UserData;
        force = [handles.forceXEdit.Value, handles.forceYEdit.Value, handles.forceZEdit.Value];
        grabbing_points = data.positions(data.selected & ~data.fixed,:);
        fixed_points = data.positions(data.fixed,:);
        differences = data.agentPoses - grabbing_points;
        dists = sqrt(sum(differences.^2, 2));
        [~, idx] = min(dists);
        data.agentPoses(idx,:) = data.agentPoses(idx,:) + force * data.dt;
        prev_positions = data.positions;
        data.positions = simulate_object_arap(arap_params, data.positions', data.agentPoses')';
        if data.recordJacobian
            actions = zeros(size(data.agentPoses,1)*3,1);
            actions(3*(idx-1)+1:3*(idx-1)+3) = force;
            data.agentActions = [data.agentActions; actions'];
            data.particleDisplacements = cat(3, data.particleDisplacements, data.positions - prev_positions);
        end
        fig.UserData = data;
        updatePlot();
        if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
            updateVonMises();
        end
    end

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

    function resetObject(~, ~)
        data = fig.UserData;
        data.positions = data.initialPositions;
        data.selected(:) = false;
        data.fixed(:) = false;
        data.agentPoses = [ 0.5,  0.5,  1.0;...
                            0.5, -0.5,  1.0;...
                           -0.5, -0.5,  1.0;...
                           -0.5,  0.5,  1.0;...
                            0.5,  0.5,  0.0;...
                            0.5, -0.5,  0.0;...
                           -0.5, -0.5,  0.0;...
                           -0.5,  0.5,  0.0];
        fig.UserData = data;
        updatePlot();
        % if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
        %     updateVonMises();
        % end
    end

    function keyPress(~, event)
        data = fig.UserData;
        if strcmp(event.Key, 'shift')
            data.shiftPressed = true;
            fig.UserData = data;
        end
    end

    function keyRelease(~, event)
        data = fig.UserData;
        if strcmp(event.Key, 'shift')
            data.shiftPressed = false;
            fig.UserData = data;
        end
    end

    function toggleVonMises(src, ~)
        data = fig.UserData;
        if strcmp(src.Value, "on")
            fig.Position = [100 100 1200 600];
            data.ax.Position = [650, 200, 500, 400];
            data.ax_vm = uiaxes(fig, 'Position', [50 200 500 400], 'Box', 'on', 'GridLineStyle', '-');
            data.ax_vm.XGrid = 'on';
            data.ax_vm.YGrid = 'on';
            data.ax_vm.ZGrid = 'on';
            xlabel(data.ax_vm, 'X');
            ylabel(data.ax_vm, 'Y');
            zlabel(data.ax_vm, 'Z');
            h = data.handles;
            offset = 400;
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
            centerOffset = 200;
            h.youngEdit.Position(1) = 460 + centerOffset;
            h.poissonEdit.Position(1) = 460 + centerOffset;
            h.youngLabel.Position(1) = 460 + centerOffset;
            h.poissonLabel.Position(1) = 460 + centerOffset;
            h.line2.Position(1) = 450 + centerOffset;
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
            h.fixButton.Position(1) = 25;
            h.thresholdEdit.Position(1) = 20;
            h.thresholdLabel.Position(1) = 20;
        else
            fig.Position = [100 100 800 600];
            data.ax.Position = [50 200 700 400];
            if isfield(data, 'ax_vm') && isvalid(data.ax_vm)
                delete(data.ax_vm);
                data = rmfield(data, 'ax_vm');
            end
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

    % +++ Deformation Methods +++
    function [deformed_vertices] = stretchFromPoint(original_vertices, grabbing_point, fixed_points, poisson_ratio, stretch_amount)
        stretch_amount = stretch_amount(:)';
        s = norm(stretch_amount);
        if s == 0
            deformed_vertices = original_vertices;
            return;
        end
        d = stretch_amount / s;
        [u, v] = getOrthonormalBasis(d);
        vec = original_vertices - grabbing_point;
        component_u = vec * u';
        component_v = vec * v';
        max_component_u = max(abs(component_u));
        max_component_v = max(abs(component_v));
        distance_to_grab = sqrt(sum((original_vertices - grabbing_point).^2, 2));
        max_distance_to_grab = max(distance_to_grab);
        weights = 1 - (distance_to_grab / (max_distance_to_grab + eps));
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
        axial_displacement = (s * weights) .* d;
        scaled_component_u = component_u / (max_component_u + eps);
        scaled_component_v = component_v / (max_component_v + eps);
        displacement_u = (-poisson_ratio * s) * weights .* scaled_component_u .* u;
        displacement_v = (-poisson_ratio * s) * weights .* scaled_component_v .* v;
        total_displacement = axial_displacement + displacement_u + displacement_v;
        deformed_vertices = original_vertices + total_displacement;
    end

    function [u, v] = getOrthonormalBasis(d)
        d = d / norm(d);
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
            stretch_amount = stretch_amount(:)';
            grabbing_center = mean(grabbing_points, 1);
            grabbing_displacement = repmat(stretch_amount, size(grabbing_points, 1), 1);
            distance_to_grab = min(vecnorm(original_vertices - permute(grabbing_points, [3, 2, 1]), 2, 2), [], 3);
            max_distance_to_grab = max(distance_to_grab) + eps;
            if isempty(fixed_points)
                min_distance_to_fixed = ones(size(original_vertices, 1), 1);
            else
                min_distance_to_fixed = min(vecnorm(original_vertices - permute(fixed_points, [3, 2, 1]), 2, 2), [], 3);
            end
            max_min_distance = max(min_distance_to_fixed) + eps;
            grab_weight = 1 - (distance_to_grab / max_distance_to_grab);
            fixed_weight = min_distance_to_fixed / max_min_distance;
            weights = grab_weight .* fixed_weight;
            total_displacement = weights .* stretch_amount;
            s = norm(stretch_amount);
            d = stretch_amount / s;
            [u, v] = getOrthonormalBasis(d);
            vec = original_vertices - grabbing_center;
            component_u = vec * u';
            component_v = vec * v';
            max_component_u = max(abs(component_u)) + eps;
            max_component_v = max(abs(component_v)) + eps;
            scaled_component_u = component_u / max_component_u;
            scaled_component_v = component_v / max_component_v;
            contraction_u = (-poisson_ratio * s) * weights .* scaled_component_u .* u;
            contraction_v = (-poisson_ratio * s) * weights .* scaled_component_v .* v;
            total_displacement = total_displacement + contraction_u + contraction_v;
            deformed_vertices = original_vertices + total_displacement;
            for i = 1:size(grabbing_points, 1)
                idx = ismember(original_vertices, grabbing_points(i, :), 'rows');
                deformed_vertices(idx, :) = grabbing_points(i, :) + stretch_amount;
            end
        else
            deformed_vertices = original_vertices;
        end
    end

    function [tetPoses, tetMeshDisplacements] = interpolateParticles(surfaceInitialPose, surfaceDisplacements, tetMeshInitPose)
        interpX = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,1), 'linear', 'nearest');
        interpY = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,2), 'linear', 'nearest');
        interpZ = scatteredInterpolant(surfaceInitialPose(:,1), surfaceInitialPose(:,2), surfaceInitialPose(:,3), surfaceDisplacements(:,3), 'linear', 'nearest');
        tetMeshDisplacements = zeros(size(tetMeshInitPose, 1), 3);
        tetMeshDisplacements(:,1) = interpX(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));
        tetMeshDisplacements(:,2) = interpY(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));
        tetMeshDisplacements(:,3) = interpZ(tetMeshInitPose(:,1), tetMeshInitPose(:,2), tetMeshInitPose(:,3));
        tetPoses = tetMeshInitPose + tetMeshDisplacements;
    end

    % +++ Von Mises +++
    function [tet_VMSigma, tet_Stress] = Compute3DvonMises(VertexInitPoses, VertexFinalPoses, MeshTetrahedrons, E, nu)
        n_nodes = length(VertexInitPoses);
        u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));
        Ce = Ce3DMatrixComputation(E, nu);
        Le = Compute3DBe(MeshTetrahedrons, VertexInitPoses);
        for elem = 1:size(MeshTetrahedrons, 1)
            nodes = MeshTetrahedrons(elem, :);
            for node = 1:4
                u_hat_e(3*node-2,1,elem) = VertexFinalPoses(nodes(node),1) - VertexInitPoses(nodes(node),1);
                u_hat_e(3*node-1,1,elem) = VertexFinalPoses(nodes(node),2) - VertexInitPoses(nodes(node),2);
                u_hat_e(3*node,1,elem)   = VertexFinalPoses(nodes(node),3) - VertexInitPoses(nodes(node),3);
            end
        end
        strains_disp = pagemtimes(Le, u_hat_e);
        stresses_disp = pagemtimes(Ce, strains_disp);
        tet_Stress = zeros(6,1,n_nodes);
        node_contributions = zeros(n_nodes, 1);
        for elem = 1:size(MeshTetrahedrons, 1)
            nodes = MeshTetrahedrons(elem, :);
            stress_elem = stresses_disp(:,:,elem);
            for i = 1:4
                tet_Stress(:,:,nodes(i)) = tet_Stress(:,:,nodes(i)) + stress_elem;
                node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
            end
        end
        for node = 1:n_nodes
            if node_contributions(node) > 0
                tet_Stress(:,:,node) = tet_Stress(:,:,node) / node_contributions(node);
            end
        end
        tet_VMSigma = zeros(n_nodes, 1);
        for node = 1:n_nodes
            sigma_12 = tet_Stress(1,:,node) - tet_Stress(2,:,node);
            sigma_23 = tet_Stress(2,:,node) - tet_Stress(3,:,node);
            sigma_31 = tet_Stress(3,:,node) - tet_Stress(1,:,node);
            tet_VMSigma(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
                3 * (tet_Stress(4,:,node)^2 + tet_Stress(5,:,node)^2 + tet_Stress(6,:,node)^2));
        end
    end

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
            indices = MeshTetraedrons(e, :);
            P1 = VertexPose(indices(1), :);
            P2 = VertexPose(indices(2), :);
            P3 = VertexPose(indices(3), :);
            P4 = VertexPose(indices(4), :);
            grads = compute3DShapeFunctionGradients(P1, P2, P3, P4);
            for i = 1:4
                B(1, 3*i-2, e) = grads(i ,1);
                B(2, 3*i-1, e) = grads(i, 2);
                B(3, 3*i, e)   = grads(i, 3);
                B(4, 3*i-2, e) = grads(i, 2);
                B(4, 3*i-1, e) = grads(i, 1);
                B(5, 3*i-1, e) = grads(i, 3);
                B(5, 3*i, e)   = grads(i, 2);
                B(6, 3*i-2, e) = grads(i, 3);
                B(6, 3*i, e)   = grads(i, 1);
            end
        end
    end

    function grads = compute3DShapeFunctionGradients(P1, P2, P3, P4)
        J = [P2 - P1; P3 - P1; P4 - P1]';
        detJ = det(J);
        if abs(detJ) < 1e-12
            error('Jacobian is singular or nearly singular. Check input points.');
        end
        invJ = inv(J);
        local_grads = [-1, -1, -1; 1, 0, 0; 0, 1, 0; 0, 0, 1];
        grads = (invJ' * local_grads')';
    end
end