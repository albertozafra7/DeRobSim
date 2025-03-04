function arap_params = create_params_for_arap(mesh,handled_nodes_ind,fixed_nodes_ind,n_iters)

%Creates a struct variable containing the parameters needed to simulate 
%the shape dynamics of a deformable object using ARAP

%Handled nodes are those that we can move (i.e., impose their position
%externally), fixed nodes are those fixed in the environment (i.e., static)

np=size(mesh.shape,1);
ele=mesh.elements;

%Compute edges between nodes from the elements of the mesh and add them to the structure
if size(ele,2)==3 %elements are triangles
    edg = [ele(:,[1 2]);ele(:,[1 3]);ele(:,[2 3])];
    
elseif size(ele,2)==4 %elements are tetrahedra
    edg = [ele(:,[1 2]); ele(:,[1 3]); ele(:,[1 4]); ...
           ele(:,[2 3]); ele(:,[2 4]); ele(:,[3 4])];    
end
edg = unique(edg,'rows');

[L,W]=arap_weighted_laplacian(mesh);
constrained_nodes_ind=[handled_nodes_ind fixed_nodes_ind];

%Determine indexes of free (i.e., not constrained) nodes
free_nodes_ind=setxor(constrained_nodes_ind,1:np);

%Laplacian matrix blocks for subsets of nodes
Li=L(free_nodes_ind,free_nodes_ind);
Le=L(free_nodes_ind,constrained_nodes_ind);

% if min(eig(Li))>0 %Matrix is definite positive
%     Fi = chol(Li); %if we can use Cholesky factorization the system is solved faster
%     arap_params.Fi=Fi;
% end

if length(constrained_nodes_ind)<1
    %We add center of mass preservation as constraint (otherwise system is indeterminate)
    Li=[Li;(1/np)*ones(1,np)];
end

Ms=(Li'*Li)\Li'; %general expression

%%%%%%%%%%%%%%%%%%%%
%Assign the values to the parameters
arap_params.rest_shape=mesh.shape;
arap_params.ele=ele;
arap_params.edg=edg;
arap_params.np=np;
arap_params.L=L;
arap_params.Li=Li;
arap_params.Le=Le;
arap_params.Ms=Ms;
arap_params.W=W;
arap_params.handled_nodes_ind=handled_nodes_ind;
arap_params.fixed_nodes_ind=fixed_nodes_ind;
arap_params.constrained_nodes_ind=constrained_nodes_ind;
arap_params.free_nodes_ind=free_nodes_ind;
arap_params.niters=n_iters;