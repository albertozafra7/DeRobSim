function [L,W]=arap_weighted_laplacian(mesh)

%L: Laplacian matrix of the mesh
%W: Weighted adjacency matrix of the mesh

shape=mesh.shape';
elem=mesh.elements;

np=size(shape,2);
W=zeros(np,np); 
ns=zeros(np,np);

%Compute edges from the elements of the mesh
if size(elem,2)==3 %elements are triangles
    edg = [elem(:,[1 2]);elem(:,[1 3]);elem(:,[2 3])];
    
elseif size(elem,2)==4 %elements are tetrahedra
    edg = [elem(:,[1 2]); elem(:,[1 3]); elem(:,[1 4]); ...
           elem(:,[2 3]); elem(:,[2 4]); elem(:,[3 4])];    
end
    
for i=1:size(edg,1)
   
    ns(edg(i,1),edg(i,2)) = 1 ;
    ns(edg(i,2),edg(i,1)) = 1 ;
    
end

%Compute W
for i=1:np
    for j=1:np
        if ns(i,j)==1 && i~=j %if there is an edge between i and j, we compute the value of its weight next   
            for k=1:np
                if ns(i,k)==1 && ns(j,k)==1 && i~=k && j~=k                    
                    
                    v1 = shape(:,i)-shape(:,k);
                    v2 = shape(:,j)-shape(:,k);
                    ang = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
                    W(i,j) = W(i,j) + 0.5 * cot(ang);
                    
                end                
            end         
        end
    end
end

D=zeros(np,np); %Degree matrix
for i=1:np
    D(i,i)=sum(W(i,:));
end

L=D-W; %Laplacian matrix