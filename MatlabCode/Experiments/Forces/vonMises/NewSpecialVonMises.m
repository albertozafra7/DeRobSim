CorrectVonMises;

%%
% [special_stresses, ~] = computeStressesByDeformationGradient(MeshTetrahedrons,VertexInitPoses,VertexFinalPoses,E,nu);


% % Initialize node-based stress tensor
% stress_n_disp_special = zeros(6,1,n_nodes);
% node_contributions = zeros(n_nodes, 1); % Track contributions per node
% 
% % Accumulate stresses for each node
% for elem = 1:size(MeshTetrahedrons, 1)
%     nodes = MeshTetrahedrons(elem, :);
%     stress_elem = special_stresses(:,elem);
%     for i = 1:4
%         stress_n_disp_special(:,:,nodes(i)) = stress_n_disp_special(:,:,nodes(i)) + stress_elem;
%         node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
%     end
% end
% 
% % Average stress at each node
% for node = 1:n_nodes
%     if node_contributions(node) > 0
%         stress_n_disp_special(:,:,node) = stress_n_disp_special(:,:,node) / node_contributions(node);
%     end
% end
% 
% % Compute von Mises stress at each node
% VMSigma_n_Disp = zeros(n_nodes, 1);
% for node = 1:n_nodes
%     sigma_12 = stress_n_disp_special(1,:,node) - stress_n_disp_special(2,:,node); % sigma_x - sigma_y
%     sigma_23 = stress_n_disp_special(2,:,node) - stress_n_disp_special(3,:,node); % sigma_y - sigma_z
%     sigma_31 = stress_n_disp_special(3,:,node) - stress_n_disp_special(1,:,node); % sigma_z - sigma_x
%     VMSigma_n_Disp(node) = sqrt(0.5 * (sigma_12^2 + sigma_23^2 + sigma_31^2) + ...
%         3 * (stress_n_disp_special(4,:,node)^2 + stress_n_disp_special(5,:,node)^2 + stress_n_disp_special(6,:,node)^2));
% end

%% New New
[special_stresses, ~,~,~] = computeStressesByDeformationGradientv2(MeshTetrahedrons,VertexInitPoses,VertexFinalPoses,Ce);
[VMSigma_n_Disp,~,stress_n_disp_special,~] = VonMisesStressComp(special_stresses,MeshTetrahedrons,n_nodes);

%% 
% Plot von Mises stress
figure;
pdeviz(R.Mesh, VMSigma_n_Disp);
title("von Mises Stress Special Data");

% Compare with result stress and compute relative error
Result_Stress = [R.Stress.sxx, R.Stress.syy, R.Stress.szz, R.Stress.sxy, R.Stress.syz, R.Stress.szx];
stressError = abs(squeeze(stress_n_disp_special)' - Result_Stress);% ./ max(Result_Stress, [], 'all');
disp("Maximum Absolute Stress Error - Special Data:");
disp(max(stressError));

% Compare with result von mises stress and compute relative error
VMError = abs(VMSigma_n_Disp - R.VonMisesStress) ./ R.VonMisesStress;
disp(strcat("Maximum Relative von Mises Error - Custom Data: ", num2str(max(VMError))));
disp(strcat("Maximum Porcentual von Mises Error - Custom Data:",num2str(max(VMError) * 100), "%"));