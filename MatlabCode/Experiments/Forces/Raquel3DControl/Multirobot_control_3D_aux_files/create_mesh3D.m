function smesh=create_mesh3D(nx,ny,nz,sx,sy,sz)


%Create a rectangular mesh for a sheet-like object
%nx, ny and nz are numbers of nodes, sx, sy and sz are dimensions in meters 

[x,y,z] = meshgrid( linspace(0,sy,ny), linspace(0,sx,nx), linspace(0,sz,nz));
smesh.shape = [x(:) y(:) z(:)];
centroid = (1/(nx*ny*nz)) * ones(1, nx*ny*nz) * smesh.shape;
smesh.shape = smesh.shape - (1/(nx*ny*nz)) * ones(1, nx*ny*nz) * smesh.shape; %to center the shape in (0,0,0)

if nx>1 && ny>1 && nz>1
    DT = delaunayTriangulation(smesh.shape(:,1),smesh.shape(:,2),smesh.shape(:,3));
    smesh.elements = DT.ConnectivityList;
    % smesh.elements = delaunay(smesh.shape(:,1:2));
else %the object is linear
    smesh.elements=[];
    for i=1:nx*ny-2
        smesh.elements=[smesh.elements;[i i+1 i+2]];
    end
end