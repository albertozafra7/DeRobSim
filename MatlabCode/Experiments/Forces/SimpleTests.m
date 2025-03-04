%% Cube Test 1
d = [-1 1];
[x,y,z] = meshgrid(d,d,d);  % A cube
x = [x(:)];
y = [y(:)];
z = [z(:)];
% [x,y,z] are corners of a cube plus the center.
VertexPose = [x(:) y(:) z(:)];
z(end-4:end) = z(end-4:end) + 1;

VertexEndPose = [x(:) y(:) z(:)];
InitVertexPose = VertexPose;

[MeshTetrahedrons,volumes] = GenerateTetrahedrons(VertexPose);

[fe,fext,decoupled_fe] = ForcesComputation(VertexEndPose, InitVertexPose, InitVertexPose, MeshTetrahedrons, volumes, 1, 1, 0.25);

% Plot the evolution of the deformations
plotVerticesComp(InitVertexPose,VertexEndPose,["Original Cube";"Deformed Cube (Stretching)"]);

% Plot the evolution of the deformations with tetramesh
plotMeshComp(InitVertexPose,VertexEndPose, MeshTetrahedrons, ["Original Cube";"Deformed Cube (Stretching)"]);

plotForces(VertexPose,MeshTetrahedrons,fe,fext,"internal");
for n=1:length(VertexPose)
    text(VertexPose(n,1),VertexPose(n,2),VertexPose(n,3),[num2str(n)]);
end

figure;
tetramesh(MeshTetrahedrons, VertexPose, 'FaceAlpha', 0.3);
hold on;
axis padded;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Tetrahedral Mesh with decoupled Forces');
for e=1:length(MeshTetrahedrons)
    for n=1:4
        quiver3(VertexPose(MeshTetrahedrons(e,n),1), VertexPose(MeshTetrahedrons(e,n),2), VertexPose(MeshTetrahedrons(e,n),3), ...
            decoupled_fe(n,1,e), decoupled_fe(n,2,e), decoupled_fe(n,3,e), ...
            0, 'b', 'LineWidth', 1.5, 'DisplayName', 'Internal Forces');
    end
end

for n=1:length(VertexPose)
    text(VertexPose(n,1),VertexPose(n,2),VertexPose(n,3),[num2str(n)]);
end
grid on;
