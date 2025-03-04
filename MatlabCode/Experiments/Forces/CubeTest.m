%% Generate a cube and generate tetrahedrons
d = [-1 1];
[x,y,z] = meshgrid(d,d,d);  % A cub
x = [x(:); 0];
y = [y(:); 0];
z = [z(:); 0];
% [x,y,z] are corners of a cube plus the center.
VertexPose = [x(:) y(:) z(:)];
VertexEndPose = [x(:) y(:) (z(:)+1)];
InitVertexPose = VertexPose;

[MeshTetrahedrons,volumes] = GenerateTetrahedrons(VertexPose);

[fe,fext, ~, stresses] = ForcesComputation(VertexEndPose, InitVertexPose, InitVertexPose, MeshTetrahedrons, volumes, 1, 1, 0.25);

% plotForces(VertexPose,MeshTetrahedrons,fe,fext);

[VMSigma_n, VMsigma_e] = VonMisesStressComp(stresses, MeshTetrahedrons, length(VertexEndPose));

plotVonMisesStress(MeshTetrahedrons,VertexPose,VMSigma_n);
