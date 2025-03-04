function new_shape=simulate_object_arap(arap_params,shape,handled_nodes_new_pos)

%Simulates the deformation of the "object" in a quasi-static scenario
%using the as-rigid-as-possible algorithm (Sorkine and Alexa, 2007)
%Updates the positions of the input "shape" to the output "new_shape"
%The handled nodes are kept in their input positions "handled_nodes_new_pos"

new_shape=shape;

if size(arap_params.handled_nodes_ind,1)>0
    new_shape(:,arap_params.handled_nodes_ind)=handled_nodes_new_pos;
end

bcentr=[];
if length(handled_nodes_new_pos)<1
       bcentr=(1/arap_params.np)*new_shape*ones(arap_params.np,1);
end
        
for iters=1:arap_params.niters

    %Right hand side
    b=arap_rhs(new_shape,arap_params.rest_shape',arap_params.W,arap_params.constrained_nodes_ind);

    b_free=b(:,arap_params.free_nodes_ind);
    b_fixed=b(:,arap_params.constrained_nodes_ind);
    bs=b_free'-arap_params.Le*b_fixed';
    bs=[bs;bcentr'];
    
    %Solve linear system
    new_shape(:,arap_params.free_nodes_ind) = (arap_params.Ms*bs)';

end




