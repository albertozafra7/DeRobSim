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
    

    %% ****************** Prediction vs Standard ******************
    % If we are not in the last iteration compute the prediction error
    if iter < niters
        next_u_hat_cbf = reshape(u_hats_cbf(:,:,iter+1),[12, 1, size(u_hats_cbf(:,:,iter+1),2)]); % (3*4 x 1 x N_tet)
        next_vm_stress_cbf = vm_stresses_cbf(:,iter+1); % (N_verts)
        next_stress_tensor_cbf = reshape(elem_stresses_cbf(:,:,iter+1),[6, 1, size(elem_stresses_cbf(:,:,iter+1),2)]); % (6 x 1 x N_tet)
        next_strain_tensor_cbf = reshape(strains_cbf(:,:,iter+1),[6, 1, size(strains_cbf(:,:,iter+1),2)]); % (6 x 1 x N_tet)
    
        % Get errors
        [error_cbf.u_hat_comp, error_cbf.strain_comp, error_cbf.stress_comp, error_cbf.vm_stress_comp] = GetPredictionErrors(next_vm_stress_cbf,pred_vm_stress_cbf,next_stress_tensor_cbf,pred_stress_tensor_cbf,next_strain_tensor_cbf,pred_strain_tensor_cbf,next_u_hat_cbf,pred_u_hat_cbf,false,"Comparison CBF");
    else
        u_hat_comp_error.values = 0;
        [u_hat_comp_error.max,u_hat_comp_error.min,u_hat_comp_error.mean,u_hat_comp_error.std] = CompareValues(u_hat_comp_error.values);
        error_cbf.u_hat_comp = u_hat_comp_error;

        strain_comp_error.values = 0;
        [strain_comp_error.max,strain_comp_error.min,strain_comp_error.mean,strain_comp_error.std] = CompareValues(strain_comp_error.values);
        error_cbf.strain_comp = strain_comp_error;

        stress_comp_error.values = 0;
        [stress_comp_error.max,stress_comp_error.min,stress_comp_error.mean,stress_comp_error.std] = CompareValues(stress_comp_error.values);
        error_cbf.stress_comp = stress_comp_error;

        vm_stress_comp_error.values = 0;
        [vm_stress_comp_error.max,vm_stress_comp_error.min,vm_stress_comp_error.mean,vm_stress_comp_error.std] = CompareValues(vm_stress_comp_error.values);
        error_cbf.vm_stress_comp = vm_stress_comp_error;
    end

    %% ****************** Errors saving ******************
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
    
    %% ****************** Prediction vs Standard ******************
    % If we are not in the last iteration compute the prediction error
    if iter < niters
        next_u_hat_3D = reshape(u_hats_3D(:,:,iter+1),[12, 1, size(u_hats_3D(:,:,iter+1),2)]); % (3*4 x 1 x N_tet)
        next_vm_stress_3D = vm_stresses_3D(:,iter+1); % (N_verts)
        next_stress_tensor_3D = reshape(elem_stresses_3D(:,:,iter+1),[6, 1, size(elem_stresses_3D(:,:,iter+1),2)]); % (6 x 1 x N_tet)
        next_strain_tensor_3D = reshape(strains_3D(:,:,iter+1),[6, 1, size(strains_3D(:,:,iter+1),2)]); % (6 x 1 x N_tet)
    
        % Get errors
        [error_3D.u_hat_comp, error_3D.strain_comp, error_3D.stress_comp, error_3D.vm_stress_comp] = GetPredictionErrors(next_vm_stress_3D,pred_vm_stress_cbf,next_stress_tensor_3D,pred_stress_tensor_cbf,next_strain_tensor_3D,pred_strain_tensor_cbf,next_u_hat_3D,pred_u_hat_cbf,false,"Comparison CBF");
    else
        u_hat_comp_error.values = 0;
        [u_hat_comp_error.max,u_hat_comp_error.min,u_hat_comp_error.mean,u_hat_comp_error.std] = CompareValues(u_hat_comp_error.values);
        error_3D.u_hat_comp = u_hat_comp_error;

        strain_comp_error.values = 0;
        [strain_comp_error.max,strain_comp_error.min,strain_comp_error.mean,strain_comp_error.std] = CompareValues(strain_comp_error.values);
        error_3D.strain_comp = strain_comp_error;

        stress_comp_error.values = 0;
        [stress_comp_error.max,stress_comp_error.min,stress_comp_error.mean,stress_comp_error.std] = CompareValues(stress_comp_error.values);
        error_3D.stress_comp = stress_comp_error;

        vm_stress_comp_error.values = 0;
        [vm_stress_comp_error.max,vm_stress_comp_error.min,vm_stress_comp_error.mean,vm_stress_comp_error.std] = CompareValues(vm_stress_comp_error.values);
        error_3D.vm_stress_comp = vm_stress_comp_error;

    end

    %% ****************** Errors saving ******************
    errors_3D = [errors_3D error_3D];
end