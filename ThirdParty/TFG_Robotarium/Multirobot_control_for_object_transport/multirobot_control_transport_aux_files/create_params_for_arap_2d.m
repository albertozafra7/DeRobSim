function arap_params = create_params_for_arap_2d(mesh,handled_nodes_ind,fixed_nodes_ind,n_iters)

%Creates a struct variable containing the parameters needed to simulate 
%the shape dynamics of a deformable object using ARAP in 2D

np=size(mesh.shape,1);
c=reshape(mesh.shape',2*np,1);
S=[0 -1;1 0];
T3=kron(eye(3),S);
K=kron(eye(np)-(1/np)*ones(np,1)*ones(1,np),eye(2));
K3=kron(eye(3)-(1/3)*ones(3,1)*ones(1,3),eye(2));
c=K*c;
T=kron(eye(np),S);
triads=mesh.elements;
ntriads=size(triads,1);

%Compute weighted Laplacian matrix of the model
Lgm=zeros(np,np);
Lg=zeros(np,np);
for m=1:size(triads,1)
    for i=1:np
        for j=1:np
          if i==j && (triads(m,1)==i || triads(m,2)==i || triads(m,3)==i)
            Lgm(i,i)=2/3;
          elseif i~=j && (triads(m,1)==i || triads(m,2)==i || triads(m,3)==i) && (triads(m,1)==j || triads(m,2)==j || triads(m,3)==j)
            Lgm(i,j)=-1/3;
          else
            Lgm(i,j)=0;
          end
        end
    end    
Lg=Lg+Lgm;
end

constrained_nodes_ind=[handled_nodes_ind,fixed_nodes_ind]';
free_nodes_ind=setxor(constrained_nodes_ind,1:np)';
Lgi=Lg(free_nodes_ind,free_nodes_ind);
Lge=Lg(free_nodes_ind,constrained_nodes_ind);

%Matrix for ARAP linear system
Ms=inv(Lgi'*Lgi)*Lgi';

%Assign the fields of the arap_params struct
arap_params.mesh=mesh;
arap_params.Ms=Ms;
arap_params.handled_nodes_ind=handled_nodes_ind;
arap_params.fixed_nodes_ind=fixed_nodes_ind;
arap_params.free_nodes_ind=free_nodes_ind;
arap_params.n_iters=n_iters;
arap_params.Lge=Lge;