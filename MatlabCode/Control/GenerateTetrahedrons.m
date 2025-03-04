% Computation of the tetrahedrons' pairs and its volumes based on the particles/vertices
% collected from the Unity's simulation.

% This function is just a preprocessing step of the internal forces
% computation
function [meshNodes, tetrahedrons, Volumes] = GenerateTetrahedrons(vertex_Initpose, elements)

    % For computing the tetrahedrons we just apply the matlab PDE
    % generateMesh method
    gm = fegeometry(vertex_Initpose, elements);
    model = femodel(AnalysisType="structuralStatic", Geometry=gm);
    model.MaterialProperties = materialProperties(YoungsModulus=E, PoissonsRatio=nu, MassDensity=2.7e-6);
    model = generateMesh(model, GeometricOrder="linear");
    mesh_model = model.Geometry.Mesh;

    % We extract the nodes and the tetrahedrons of the mesh
    meshNodes = mesh_model.Nodes';
    tetrahedrons = mesh_model.Elements';

    % For computing each tetrahedron's volume the standard formula found in
    % https://es.mathworks.com/matlabcentral/answers/316266-how-i-can-calculate-area-and-volume-if-have-4-coordinates-like-x1-y1-x2-y2-x3-y3-x4-y4
    Volumes = abs(dot(cross(meshNodes(tetrahedrons(:,2),:) - meshNodes(tetrahedrons(:,1),:), ...
                            meshNodes(tetrahedrons(:,3),:) - meshNodes(tetrahedrons(:,1),:), 2), ...
                            meshNodes(tetrahedrons(:,4),:) - meshNodes(tetrahedrons(:,1),:), 2)) / 6;


end