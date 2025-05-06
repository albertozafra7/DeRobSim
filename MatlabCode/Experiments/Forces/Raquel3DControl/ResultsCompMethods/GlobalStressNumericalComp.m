%%%%%%%%%%%%%%%%%%%%%%% Global Stress Comparisons %%%%%%%%%%%%%%%%%%%%%%%
errors_cbf = [];
errors_3D = [];

for iter = 1:niters
    %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% CBF %%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%
    % ****************** Standard ******************
    % Data loading
    vm_stress_cbf = vm_stresses_cbf(:,iter); % (N_verts)
    stress_tensor_cbf = reshape(elem_stresses_cbf(:,:,iter),[6, 1, size(elem_stresses_cbf(:,:,iter),2)]); % (6 x 1 x N_tet)
    strain_tensor_cbf = reshape(strains_cbf(:,:,iter),[6, 1, size(strains_cbf(:,:,iter),2)]); % (6 x 1 x N_tet)
    u_hat_cbf = reshape(u_hats_cbf(:,:,iter),[12, 1, size(u_hats_cbf(:,:,iter),2)]); % (3*4 x 1 x N_tet)
    
    % Equivalence evaluation
    [error_cbf.strain, error_cbf.stress, error_cbf.vm_stress] = GetFEMErrors(vm_stress_cbf,stress_tensor_cbf,strain_tensor_cbf,u_hat_cbf,Le_cbf,Ce_cbf,omesh,false, "Standard CBF");
    
    
    %% ****************** Prediction ******************
    
    % Data loading
    pred_vm_stress_cbf = pred_vmStresses_cbf(:,iter); % (N_verts)
    pred_stress_tensor_cbf = reshape(pred_elem_stresses_cbf(:,:,iter),[6, 1, size(pred_elem_stresses_cbf(:,:,iter),2)]); % (6 x 1 x N_tet)
    pred_strain_tensor_cbf = reshape(pred_strains_cbf(:,:,iter),[6, 1, size(pred_strains_cbf(:,:,iter),2)]); % (6 x 1 x N_tet)
    pred_u_hat_cbf = reshape(element_disps_cbf(:,:,iter),[12, 1, size(element_disps_cbf(:,:,iter),2)]); % (3*4 x 1 x N_tet)
    
    % Equivalence evaluation
    [error_cbf.pred_strain, error_cbf.pred_stress, error_cbf.pred_vm_stress] = GetFEMErrors(pred_vm_stress_cbf,pred_stress_tensor_cbf,pred_strain_tensor_cbf,pred_u_hat_cbf,Le_cbf,Ce_cbf,omesh,false,"Predicted CBF");
    
    % If we are not in the last iteration compute the predicted u hat error
    if iter < niters
        next_u_hat_cbf = reshape(u_hats_cbf(:,:,iter+1),[12, 1, size(u_hats_cbf(:,:,iter+1),2)]); % (3*4 x 1 x N_tet)
        pred_u_hat_error.values = abs(next_u_hat_cbf - pred_u_hat_cbf);
    else
        pred_u_hat_error.values = 0;
    end
    [pred_u_hat_error.max,pred_u_hat_error.min,pred_u_hat_error.mean,pred_u_hat_error.std] = CompareValues(pred_u_hat_error.values);
    error_cbf.pred_u_hat = pred_u_hat_error;

    errors_cbf = [errors_cbf error_cbf];
    %%
    %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% 3D %%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%

    % ****************** Standard ******************

    % Data loading
    vm_stress_3D = vm_stresses_3D(:,iter); % (N_verts)
    stress_tensor_3D = reshape(elem_stresses_3D(:,:,iter),[6, 1, size(elem_stresses_3D(:,:,iter),2)]); % (6 x 1 x N_tet)
    strain_tensor_3D = reshape(strains_3D(:,:,iter),[6, 1, size(strains_3D(:,:,iter),2)]); % (6 x 1 x N_tet)
    u_hat_3D = reshape(u_hats_3D(:,:,iter),[12, 1, size(u_hats_3D(:,:,iter),2)]); % (3*4 x 1 x N_tet)
    
    % Equivalence evaluation
    [error_3D.strain, error_3D.stress, error_3D.vm_stress] = GetFEMErrors(vm_stress_3D,stress_tensor_3D,strain_tensor_3D,u_hat_3D,Le_3D,Ce_3D,omesh,false, "Standard 3D");
    
    %% ****************** Prediction ******************

    % Data loading
    pred_vm_stress_3D = pred_vmStresses_3D(:,iter); % (N_verts)
    pred_stress_tensor_3D = reshape(pred_elem_stresses_3D(:,:,iter),[6, 1, size(pred_elem_stresses_3D(:,:,iter),2)]); % (6 x 1 x N_tet)
    pred_strain_tensor_3D = reshape(pred_strains_3D(:,:,iter),[6, 1, size(pred_strains_3D(:,:,iter),2)]); % (6 x 1 x N_tet)
    pred_u_hat_3D = reshape(element_disps_3D(:,:,iter),[12, 1, size(element_disps_3D(:,:,iter),2)]); % (3*4 x 1 x N_tet)
    
    % Equivalence evaluation
    [error_3D.pred_strain, error_3D.pred_stress, error_3D.pred_vm_stress] = GetFEMErrors(pred_vm_stress_3D,pred_stress_tensor_3D,pred_strain_tensor_3D,pred_u_hat_3D,Le_3D,Ce_3D,omesh,false,"Predicted 3D");
    
    % If we are not in the last iteration compute the predicted u hat error
    if iter < niters
        next_u_hat_3D = reshape(u_hats_3D(:,:,iter+1),[12, 1, size(u_hats_3D(:,:,iter+1),2)]); % (3*4 x 1 x N_tet)
        pred_u_hat_error.values = abs(next_u_hat_3D - pred_u_hat_3D);
    else
        pred_u_hat_error.values = 0;
    end
    [pred_u_hat_error.max,pred_u_hat_error.min,pred_u_hat_error.mean,pred_u_hat_error.std] = CompareValues(pred_u_hat_error.values);
    error_3D.pred_u_hat = pred_u_hat_error;

    % Accumulate all errors
    errors_3D = [errors_3D error_3D];
end