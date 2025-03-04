function b=arap_rhs(pos,pos0,W,constr)

%Computes right-hand-side of the linear system of equations for arap
%pos: current positions
%pos0: positions in rest shape
%W: weight matrix
%constr: indexes of the nodes whose positions are constrained externally

np=size(pos,2);
b=zeros(3,np);
Rs=zeros(3,3*np);


%Compute optimal rotation matrices for each cell in the mesh
for i=1:np
    
    js=find(W(i,:));
    wm=diag(W(i,js));
    ls=length(js);
  
    Pi0 = (pos0(:,i)*ones(1,ls) - pos0(:,js));
    Pi =  (pos(:,i)*ones(1,ls) - pos(:,js));    
    [uu,ee,vv] = svd( Pi0 * wm * Pi');
    Rs(:,3*(i-1)+1:3*i)=vv*[1 0 0; 0 1 0; 0 0 sign(det(vv*uu'))]*uu';
    
end

%Compute rhs values
for i=1:np
    
    if isempty(find(constr==i,1))
        
        for j=1:np 

            if abs(W(i,j))>1e-7 && j~=i 

                b(:,i)=b(:,i)+0.5*W(i,j)*(Rs(:,3*(i-1)+1:3*i)+Rs(:,3*(j-1)+1:3*j))*(pos0(:,i)-pos0(:,j));

            end
            
        end
        
    end    
    
end

%Positions of constrained nodes are externally fixed
b(:,constr)=pos(:,constr);