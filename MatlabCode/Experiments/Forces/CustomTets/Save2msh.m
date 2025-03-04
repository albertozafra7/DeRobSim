% Generate a mesh using MATLAB
% model = createpde();
% geometryFromEdges(model, @circleg);
% mesh = generateMesh(model);

% Extract nodes and elements
nodes = mesh_model.Nodes; % Node coordinates
elements = mesh_model.Elements; % Element connectivity

% Open the file for writing
fileID = fopen('mesh_with_bbox.msh', 'w');

% Compute bounding box
boundingBox.Min = min(nodes, [], 2); % Minimum bounds (X, Y, Z)
boundingBox.Max = max(nodes, [], 2); % Maximum bounds (X, Y, Z)

% Write MeshFormat section
fprintf(fileID, '$MeshFormat\n');
fprintf(fileID, '4.1 0 8\n');
fprintf(fileID, '$EndMeshFormat\n');

% Write Entities section
fprintf(fileID, '$Entities\n');
% In this example, we assume one surface entity and one volume entity
numPoints = 0; % Number of point entities
numCurves = 0; % Number of curve entities
numSurfaces = 1; % Number of surface entities
numVolumes = 1; % Number of volume entities

fprintf(fileID, '%d %d %d %d\n', numPoints, numCurves, numSurfaces, numVolumes);

% Write surface entity
fprintf(fileID, '1 %.15g %.15g %.15g %.15g %.15g %.15g 0 0\n', ...
    boundingBox.Min(1), boundingBox.Min(2), boundingBox.Min(3), ...
    boundingBox.Max(1), boundingBox.Max(2), boundingBox.Max(3));

% Write volume entity
fprintf(fileID, '2 %.15g %.15g %.15g %.15g %.15g %.15g 0 0\n', ...
    boundingBox.Min(1), boundingBox.Min(2), boundingBox.Min(3), ...
    boundingBox.Max(1), boundingBox.Max(2), boundingBox.Max(3));

fprintf(fileID, '$EndEntities\n');


% Write Nodes section
numNodes = size(nodes, 2);
fprintf(fileID, '$Nodes\n');
fprintf(fileID, '2 %d 1 %d\n', numNodes, numNodes); % 2 entity nodes
fprintf(fileID, '2 1 0 8\n'); % Example node block header

% Write node tags and coordinates
for i = 1:numNodes
    fprintf(fileID, '%d\n', i); % Node tag
end
for i = 1:numNodes
    if size(nodes, 1) == 2
        fprintf(fileID, '%.15g %.15g 0.0\n', nodes(1, i), nodes(2, i));
    else
        fprintf(fileID, '%.15g %.15g %.15g\n', nodes(1, i), nodes(2, i), nodes(3, i));
    end
end
fprintf(fileID, '$EndNodes\n');

% Write Elements section
numElements = size(elements, 2);
fprintf(fileID, '$Elements\n');
fprintf(fileID, '2 %d 1 %d\n', numElements, numElements); % 2 entity elements

% Write element data
for i = 1:numElements
    elem = elements(:, i);
    elemType = length(elem); % Determine type (triangle, tetrahedron, etc.)
    if elemType == 3
        gmshElemType = 2; % Triangle
    elseif elemType == 4
        gmshElemType = 4; % Tetrahedron
    else
        error('Unsupported element type');
    end
    fprintf(fileID, '%d', i);
    fprintf(fileID, ' %d', elem);
    fprintf(fileID, ' \n');
end
fprintf(fileID, '$EndElements\n');

% Close the file
fclose(fileID);
