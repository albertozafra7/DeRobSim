function shape = simulate_object_arap_2d(arap_params,shape,handled_nodes_new_pos)

%Simulation of ARAP in 2D
%Based on grouping the nodes in triads

%Define parameters
ntriads=size(arap_params.mesh.elements,1);
np=size(shape,2);
Lge=arap_params.Lge;
Ms=arap_params.Ms;
triads=arap_params.mesh.elements;
free_nodes_ind=arap_params.free_nodes_ind;
handled_nodes_ind=arap_params.handled_nodes_ind;
shape(:,handled_nodes_ind)=handled_nodes_new_pos;
S=[0 -1;1 0];
T3=kron(eye(3),S);
K3=kron(eye(3)-(1/3)*ones(3,1)*ones(1,3),eye(2));

%Reshape the current and rest shapes as 2n x 1 vectors
q=reshape(shape,2*np,1);
c=reshape(arap_params.mesh.shape',2*np,1);

for k=1:arap_params.n_iters

    %Compute optimal rotation for every "cell" (i.e., "triad")
    Rs=[];
    for i=1:ntriads

            ind=[2*triads(i,1)-1;2*triads(i,1);2*triads(i,2)-1;...
                 2*triads(i,2);2*triads(i,3)-1;2*triads(i,3)];
            qt=q(ind);
            ct=K3*c(ind);
            ai=atan2(qt'*T3*ct,qt'*ct);
            Rs=[Rs,[cos(ai) -sin(ai); sin(ai) cos(ai)]];

    end

    %Compute the right-hand-side of the ARAP equation
    b=zeros(2,np);
    for i=1:np    
        for j=1:ntriads
            if (triads(j,1)==i || triads(j,2)==i || triads(j,3)==i)

               indj=[2*triads(j,1)-1;2*triads(j,1);2*triads(j,2)-1;...
                     2*triads(j,2);2*triads(j,3)-1;2*triads(j,3)];
               ctj=K3*c(indj);
               ctj2=reshape(ctj,2,3);

               Rj=Rs(:,2*j-1:2*j);
               if triads(j,1)==i 
                 b(:,i)=b(:,i)+Rj*ctj2(:,1);
               elseif triads(j,2)==i
                 b(:,i)=b(:,i)+Rj*ctj2(:,2);       
               elseif triads(j,3)==i 
                 b(:,i)=b(:,i)+Rj*ctj2(:,3);
               end
            end
        end
    end


    b_free=b(:,free_nodes_ind); %positions of free nodes
    b_constr=shape(:,handled_nodes_ind); %positions of constrained nodes
    bs=b_free'-Lge*b_constr'; %Right-Hand-Side of the system we will solve

    %Solve ARAP system
    shape(:,free_nodes_ind) = (Ms*bs)';
    
    %Resulting shape after one ARAP iteration
    q=reshape(shape,2*np,1);

end