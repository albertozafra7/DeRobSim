function smesh=create_mesh(nx,ny,sx,sy)


%Create a rectangular mesh for a sheet-like object
%nx, ny are numbers of nodes, sx, sy are dimensions in meters 

[x,y] = meshgrid( linspace(0,sy,ny), linspace(0,sx,nx) );
smesh.shape = [x(:) y(:) 0.01*ones(size(x(:)))];
smesh.shape = smesh.shape-(1/(nx*ny))*ones(1,nx*ny)*smesh.shape; %to center the shape in (0,0,0)

if nx>1 && ny>1
    smesh.elements = delaunay(smesh.shape(:,1:2));
else %the object is linear
    smesh.elements=[];
    for i=1:nx*ny-2
        smesh.elements=[smesh.elements;[i i+1 i+2]];
    end
end