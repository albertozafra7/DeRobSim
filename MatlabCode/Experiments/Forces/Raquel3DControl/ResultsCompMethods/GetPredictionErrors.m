function [u_hat_error, strain_error, stress_error, vm_error] = GetPredictionErrors(curr_vm_stress,pred_vm_stress,curr_elem_stress,pred_elem_stress,curr_strain,pred_strain,curr_u_hat,pred_u_hat,DispComp,DispSufix)

    % Optional variable handling
    if ~exist("DispComp","var")
        DispComp = false;
    end
    if ~exist("DispSufix","var")
        DispSufix = "";
    end

    % Equivalence evaluation
    % u_hat
    u_hat_error.values = abs(curr_u_hat - pred_u_hat);

    % strain
    strain_error.values = abs(curr_strain - pred_strain);
    
    % strain vs stress
    stress_error.values = abs(curr_elem_stress - pred_elem_stress);
    
    % stress vs vonMises
    vm_error.values = abs(curr_vm_stress - pred_vm_stress);

    % Getting and printing the comparison results (if requested)
    [u_hat_error.max,u_hat_error.min,u_hat_error.mean,u_hat_error.std] = CompareValues(u_hat_error.values, DispComp,"u_hat",DispSufix);
    [strain_error.max,strain_error.min,strain_error.mean,strain_error.std] = CompareValues(strain_error.values, DispComp,"strain",DispSufix);
    [stress_error.max,stress_error.min,stress_error.mean,stress_error.std] = CompareValues(stress_error.values, DispComp,"stress",DispSufix);
    [vm_error.max,vm_error.min,vm_error.mean,vm_error.std] = CompareValues(vm_error.values, DispComp,"vm_stress",DispSufix);
end