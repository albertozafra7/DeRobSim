function [strain_error, stress_error, vm_error] = GetFEMErrors(vm_stress,elem_stress,strain,u_hat,Le,Ce,omesh,DispComp,DispSufix)

    % Optional variable handling
    if ~exist("DispComp","var")
        DispComp = false;
    end
    if ~exist("DispSufix","var")
        DispSufix = "";
    end

    % Equivalence evaluation
    % u_hat vs strain
    comp_strain = pagemtimes(Le,u_hat);
    strain_error.values = abs(strain - comp_strain);
    
    % strain vs stress
    comp_stress = pagemtimes(Ce,comp_strain);
    stress_error.values = abs(elem_stress - comp_stress);
    
    % stress vs vonMises
    [comp_vmStress,~] = VonMisesStressComp(comp_stress,omesh.elements,size(omesh.shape,1));
    vm_error.values = abs(vm_stress - comp_vmStress);

    % Getting and printing the comparison results (if requested)
    [strain_error.max,strain_error.min,strain_error.mean,strain_error.std] = CompareValues(strain_error.values, DispComp,"strain",DispSufix);
    [stress_error.max,stress_error.min,stress_error.mean,stress_error.std] = CompareValues(stress_error.values, DispComp,"stress",DispSufix);
    [vm_error.max,vm_error.min,vm_error.mean,vm_error.std] = CompareValues(vm_error.values, DispComp,"vm_stress",DispSufix);
end