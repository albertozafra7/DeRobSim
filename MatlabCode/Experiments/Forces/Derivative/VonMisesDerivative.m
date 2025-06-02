BeamSetUp_Transient; % With this we get a transient model and its solutions
% The properties generated are:
% E --> Young Modulus (Pa)
% nu --> Poisson Ratio (Unitless) [-0.1,0.5]
% MassDensity --> MassDensity
% R --> Standard Transient model solutions including stresses, strains, vm
% R --> Epsilon Transient model solutions including stresses, strains, vm

%% Set Up

sim_values_R = setUpSimulation(R);
sim_values_R_eps = setUpSimulation(R_eps);

%% Stress, Strains and derivatives computation
sim_values_R = computeSimulationStresses(sim_values_R);
sim_values_R_eps = computeSimulationStresses(sim_values_R_eps);

%% Error computation

sim_values_R = computeSimulationErrors(sim_values_R, R);
sim_values_R_eps = computeSimulationErrors(sim_values_R_eps, R_eps);

%% We plot the errors
plotErrorValue([sim_values_R.errors.vmStress.Porcentual],"Von Mises Porcentual Error 0:0.002:0.2",["Iter";"Pa"]);
% tensorNames = ["XX", "YY", "ZZ", "XY", "YZ", "XZ"];
% plotErrortensor(sim_values_R.errors.Stress,"Stress Error 0:0.002:0.2",["Iter";"Pa"],"Stress - ", tensorNames);
% plotErrortensor(sim_values_R.errors.Strain,"Strain Error 0:0.002:0.2",["Iter";"m"],"Strain - ", tensorNames);


plotErrorValue([sim_values_R_eps.errors.vmStress.Absolute],"Von Mises Absolute Error with epsilon increment",["Iter";"Pa"]);
% plotErrortensor(sim_values_R_eps.errors.Stress,"Stress Error with epsilon increment",["Iter";"Pa"],"Stress - ", tensorNames);
% plotErrortensor(sim_values_R_eps.errors.Strain,"Strain Error with epsilon increment",["Iter";"m"],"Strain - ", tensorNames);

%% We compute the derivative error
[sim_values_R] = computeDtiveErrors(sim_values_R);
[sim_values_R_eps] = computeDtiveErrors(sim_values_R_eps);

%% Derivative Error plot
plotErrorValue([sim_values_R.errorsDtive.vmStress.Porcentual],"Von Mises Derivative Porcentual Error 0:0.002:0.2",["Iter";"Pa"]);
tensorNames = ["XX", "YY", "ZZ", "XY", "YZ", "XZ"];
plotErrortensor(sim_values_R.errorsDtive.Stress,"Stress Error 0:0.002:0.2",["Iter";"Pa"],"Stress - ", tensorNames);
plotErrortensor(sim_values_R.errorsDtive.Strain,"Strain Error 0:0.002:0.2",["Iter";"m"],"Strain - ", tensorNames);
uHatNames = [];
for i = 1:4
    uHatNames = [uHatNames, strcat(num2str(i)," - X"), strcat(num2str(i)," - Y"), strcat(num2str(i)," - Z")];
end
plotErrortensor(sim_values_R.errorsDtive.XInter_elem,"uHat Error 0:0.002:0.2",["Iter";"m"],"uHat - ", uHatNames);
% figure;
% plot(squeeze(sim_values_R.XInter_elem(1,:,:)));

plotErrorValue([sim_values_R_eps.errorsDtive.vmStress.Absolute],"Von Mises Derivative Absolute Error with epsilon increment",["Iter";"Pa"]);
% plotErrortensor(sim_values_R_eps.errorsDtive.Stress,"Stress Error with epsilon increment",["Iter";"Pa"],"Stress - ", tensorNames);
% plotErrortensor(sim_values_R_eps.errorsDtive.Strain,"Strain Error with epsilon increment",["Iter";"m"],"Strain - ", tensorNames);

%%
% % Plot von Mises stress
% figure;
% pdeviz(R.Mesh, sim_values_R.vmStress(:,52));
% title("von Mises Stress Special Data");
% 
% figure;
% pdeviz(R.Mesh, R.vmStress(:,52));
% title("von Mises Stress PDE");

% 
% % Compare with result stress and compute relative error
% Result_Stress = [R.Stress.sxx, R.Stress.syy, R.Stress.szz, R.Stress.sxy, R.Stress.syz, R.Stress.szx];
% stressError = abs(squeeze(stress_n_disp_special)' - Result_Stress) ./ max(Result_Stress, [], 'all');
% disp("Maximum Relative Stress Error - Special Data:");
% disp(max(stressError));
% 
% Compare with result von mises stress and compute relative error
% VMError = abs(sim_values_R.vmStress(:,52) - R.vmStress(:,52)) ./ max(R.vmStress(:,52));
% disp("Maximum Relative von Mises Error - Special Data:");
% disp(max(VMError));





%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We set up the simulation values for latter computing the von Mises and
% its derivative
function [sim_values] = setUpSimulation(model)
    sim_values.N_verts = size(model.Mesh.Nodes,2);
    sim_values.N_tet = size(model.Mesh.Elements,2);
    sim_values.N_times = size(model.SolutionTimes,2);
    sim_values.E = model.E;
    sim_values.nu = model.nu;
    sim_values.kext = 10; % Stiffness off the external forces (N/m)
    sim_values.MeshTetrahedrons = model.Mesh.Elements'; % Mesh Tetrahedra

    % We save the initial and final vertex positions of the model
    sim_values.X0 = model.Mesh.Nodes'; 
    sim_values.XF = zeros(sim_values.N_verts,3);
    
    sim_values.XF(:,1) = sim_values.X0(:,1) + sum(model.Displacement.x,2);
    sim_values.XF(:,2) = sim_values.X0(:,2) + sum(model.Displacement.y,2);
    sim_values.XF(:,3) = sim_values.X0(:,3) + sum(model.Displacement.z,2);
    
    % We save the inter vertex positions of the model
    sim_values.XInter = zeros(sim_values.N_times,sim_values.N_verts,3);
    
    VertexPoses = sim_values.X0;
    for t = 1:sim_values.N_times
        VertexPoses(:,1) = sim_values.X0(:,1) + model.Displacement.x(:,t);
        VertexPoses(:,2) = sim_values.X0(:,2) + model.Displacement.y(:,t);
        VertexPoses(:,3) = sim_values.X0(:,3) + model.Displacement.z(:,t);
        
        sim_values.XInter(t,:,:) = VertexPoses;
    end

    % Compute element stiffness matrix (6x6xN_tet)
    sim_values.Ce = CeMatrixComputation(sim_values.E, sim_values.nu);
    % Compute strain-displacement matrices (6x12xN_tet)
    sim_values.Le = LeMatrixComputation(sim_values.MeshTetrahedrons, sim_values.X0);


end

function [sim_values] = computeSimulationStresses(sim_values)
    % Set Up the stress and strains matrices
    sim_values.Stress = zeros(6,sim_values.N_verts,sim_values.N_times);
    sim_values.Stress_elem = zeros(6, sim_values.N_tet,sim_values.N_times);
    sim_values.Strain_elem = zeros(6,sim_values.N_tet,sim_values.N_times);
    sim_values.Strain = zeros(6,sim_values.N_verts,sim_values.N_times);
    sim_values.vmStress = zeros(sim_values.N_verts,sim_values.N_times);
    sim_values.vmStress_elem = zeros(sim_values.N_tet,sim_values.N_times);
    sim_values.u_hat_e = zeros(12, sim_values.N_tet, sim_values.N_times);

    % Set Up the Derivatives matrices
    sim_values.vmDtive = zeros(sim_values.N_verts,sim_values.N_times);
    sim_values.XInter_elem = zeros(12,sim_values.N_tet,sim_values.N_times);
    sim_values.StrainDtive_elem = zeros(6,sim_values.N_tet,sim_values.N_times);
    sim_values.StrainDtive = zeros(6,sim_values.N_verts,sim_values.N_times);
    sim_values.StressDtive_elem = zeros(6,sim_values.N_tet,sim_values.N_times);
    sim_values.StressDtive = zeros(6,sim_values.N_verts,sim_values.N_times);

    % Compute the Stresses and strains values with their derivatives
    for t = 1:sim_values.N_times
        [sim_values.Stress_elem(:,:,t), ~, sim_values.Strain_elem(:,:,t), ~,sim_values.u_hat_e(:,:,t)] = ComputeStressByDisplacements(squeeze(sim_values.XInter(t,:,:)) - sim_values.X0,sim_values.MeshTetrahedrons, sim_values.Ce, sim_values.Le);
        % [sim_values.Stress_elem(:,:,t), sim_values.Strain_elem(:,:,t),~] = computeStressesByDeformationGradientv2(sim_values.MeshTetrahedrons,sim_values.X0,squeeze(sim_values.XInter(t,:,:)),sim_values.Ce);
        [sim_values.vmStress(:,t), sim_values.vmStress_elem(:,t), sim_values.Stress(:,:,t), ~] = VonMisesStressComp(sim_values.Stress_elem(:,:,t), sim_values.MeshTetrahedrons, sim_values.N_verts);
        [sim_values.vmDtive(:,t), XInter_elem, StrainDtive_elem, StressDtive_elem, StressDtive, sim_values.G, sim_values.Le_blck, sim_values.W_blck, sim_values.W_basic] = DerivateVonMises(sim_values.vmStress(:,t), sim_values.Ce, sim_values.Le,squeeze(sim_values.XInter(t,:,:)) - sim_values.X0, sim_values.MeshTetrahedrons);
        
        Strain = (sim_values.W_basic*squeeze(sim_values.Strain_elem(:,:,t))')./(sum(sim_values.W_basic, 2)+eps);
        sim_values.Strain(:,:,t) = Strain';

        sim_values.XInter_elem(:,:,t) = reshape(XInter_elem,[12,sim_values.N_tet]);
        StrainDtive = (sim_values.W_blck*StrainDtive_elem)./(sum(sim_values.W_blck, 2)+eps);
        sim_values.StrainDtive(:,:,t) = reshape(StrainDtive,[6,sim_values.N_verts]);
        sim_values.StrainDtive_elem(:,:,t) = reshape(StrainDtive_elem,[6,sim_values.N_tet]);
        sim_values.StressDtive_elem(:,:,t) = reshape(StressDtive_elem,[6,sim_values.N_tet]);
        sim_values.StressDtive(:,:,t) = reshape(StressDtive,[6,sim_values.N_verts]);
    end

end

%%%%%%%%%%%% Errors %%%%%%%%%%%%
function [sim_values] = computeSimulationErrors(sim_values, R)
    errors_vmStress = [];
    errors_Stress = [];
    errors_Strain = [];

    for t=2:sim_values.N_times
        e_vmStress.Absolute = computeAbsoluteError(sim_values.vmStress(:,t),R.vmStress(:,t));
        [e_vmStress.Relative, e_vmStress.Porcentual] = computePorcentualError(sim_values.vmStress(:,t),R.vmStress(:,t));
        e_Stress = computeErrorTensor(squeeze(sim_values.Stress(:,:,t)),[R.Stress.sxx(:,t), R.Stress.syy(:,t), R.Stress.szz(:,t), R.Stress.sxy(:,t), R.Stress.syz(:,t), R.Stress.szx(:,t)]);
        e_Stain = computeErrorTensor(squeeze(sim_values.Strain(:,:,t)),[R.Strain.xx(:,t), R.Strain.yy(:,t), R.Strain.zz(:,t), R.Strain.xy(:,t), R.Strain.yz(:,t), R.Strain.xz(:,t)]);
        
        errors_vmStress = [errors_vmStress, e_vmStress];
        errors_Stress = [errors_Stress, e_Stress];
        errors_Strain = [errors_Strain, e_Stain];
    end
    
    sim_values.errors.vmStress = errors_vmStress;
    sim_values.errors.Stress = errors_Stress;
    sim_values.errors.Strain = errors_Strain;
end


function [sim_values] = computeDtiveErrors(sim_values)
    error_vmStressDtive = [];
    error_StressDtive = [];
    error_StrainDtive = [];
    error_XInter_elem = [];
    error_StressDtive_elem = [];
    error_StrainDtive_elem = [];

    for t=2:sim_values.N_times
        e_vmStress.Absolute = computeAbsoluteError(sim_values.vmStress(:,t),sim_values.vmStress(:,t)-sim_values.vmStress(:,t-1));
        [e_vmStress.Relative, e_vmStress.Porcentual] = computePorcentualError(sim_values.vmStress(:,t),sim_values.vmStress(:,t)-sim_values.vmStress(:,t-1));
        e_Stress = computeErrorTensor(squeeze(sim_values.StressDtive(:,:,t)),squeeze(sim_values.Stress(:,:,t)-sim_values.Stress(:,:,t-1)));
        e_Stress_elem = computeErrorTensor(squeeze(sim_values.StressDtive_elem(:,:,t)),squeeze(sim_values.Stress_elem(:,:,t)-sim_values.Stress_elem(:,:,t-1)));
        e_Stain = computeErrorTensor(squeeze(sim_values.StrainDtive(:,:,t)),squeeze(sim_values.Strain(:,:,t)-sim_values.Strain(:,:,t-1)));
        e_Stain_elem = computeErrorTensor(squeeze(sim_values.StrainDtive_elem(:,:,t)),squeeze(sim_values.Strain_elem(:,:,t)-sim_values.Strain_elem(:,:,t-1)));
        e_XInter_elem = computeErrorTensor(squeeze(sim_values.XInter_elem(:,:,t)),squeeze(sim_values.u_hat_e(:,:,t)-sim_values.u_hat_e(:,:,t-1)));
        
        error_vmStressDtive = [error_vmStressDtive, e_vmStress];
        error_StressDtive = [error_StressDtive, e_Stress];
        error_StressDtive_elem = [error_StressDtive_elem, e_Stress_elem];
        error_StrainDtive = [error_StrainDtive, e_Stain];
        error_StrainDtive_elem = [error_StrainDtive_elem, e_Stain_elem];
        error_XInter_elem = [error_XInter_elem, e_XInter_elem];

    end
    
    sim_values.errorsDtive.vmStress = error_vmStressDtive;
    sim_values.errorsDtive.Stress = error_StressDtive;
    sim_values.errorsDtive.Stress_elem = error_StressDtive_elem;
    sim_values.errorsDtive.Strain = error_StrainDtive;
    sim_values.errorsDtive.Strain_elem = error_StrainDtive_elem;
    sim_values.errorsDtive.XInter_elem = error_XInter_elem;

end


function [error] = computeAbsoluteError(Variable,Groundtruth)
    error.value = abs(Variable - Groundtruth);
    [error.max,error.min,error.mean,error.std] = CompareValues(error.value);
end

function [error_relative,error_porcentual] = computePorcentualError(Variable,Groundtruth)

    error_relative.value = abs(Variable - Groundtruth)./(Groundtruth+eps);
    [error_relative.max,error_relative.min,error_relative.mean,error_relative.std] = CompareValues(error_relative.value);

    error_porcentual.value = error_relative.value * 100;
    [error_porcentual.max,error_porcentual.min,error_porcentual.mean,error_porcentual.std] = CompareValues(error_porcentual.value);
end


function [error] = computeErrorTensor(Tensor,Groundtruth)
    error = [];
    for i = 1:size(Tensor,1)
        if size(Tensor) ~= size(Groundtruth)
            error = [error; computeAbsoluteError(Tensor(i,:)',Groundtruth(:,i))];
        else
            error = [error; computeAbsoluteError(Tensor(i,:),Groundtruth(:,i))];
        end
    end
end

function [] = plotErrorValue(ErrVal,plotTitle,plotLabels)
    % Full values plot
    figure;
    grid on;
    plot([ErrVal(:).max],'DisplayName', "max", 'Color','r', 'LineWidth', 2);
    hold on;
    plot([ErrVal(:).min],'DisplayName', "min", 'Color','b', 'LineWidth', 2);
    plot([ErrVal(:).mean],'DisplayName', "mean", 'Color', 'black', 'LineWidth', 2, 'LineStyle',':');
    legend show;
    axis padded;
    title(plotTitle);
    xlabel(plotLabels(1));
    ylabel(plotLabels(2));

end

function [] = plotErrortensor(ErrVal,plotTitle,plotLabels,tensorPrefix, tensorNames)
    figure;
    title(plotTitle);
    for i = 1:size(ErrVal,1)
        subplot(2,size(ErrVal,1)/2,i);
        grid on;
        plot([ErrVal(i,:).max],'DisplayName', "max", 'Color','r', 'LineWidth', 2);
        hold on;
        plot([ErrVal(i,:).min],'DisplayName', "min", 'Color','b', 'LineWidth', 2);
        plot([ErrVal(i,:).mean],'DisplayName', "mean", 'Color', 'black', 'LineWidth', 2, 'LineStyle',':');
        legend show;
        axis padded;
        title(strcat(tensorPrefix,tensorNames(i)));
        xlabel(plotLabels(1));
        ylabel(plotLabels(2));

    end
end

%%%%%
function [grad_h, element_displacements, pred_strains, pred_stresses_element, pred_nodal_stresses, G, Le_blck, W_blck, W_basic] = DerivateVonMises(VM_stress, Ce, Le, nodal_displacements, meshTetrahedrons)

    N_tet = size(meshTetrahedrons, 1); % Number of tetrahedrons
    N_verts = size(VM_stress, 1);      % Number of vertices
    % N_a = size(J,2)/3;

    % -----------------------------------------------------------
    % Compute von Mises stress gradient delta

    % -----------------------------------------------------------
    % 1️⃣ Construct matrix G: maps nodal displacements to element displacements
    % -----------------------------------------------------------

    elements = meshTetrahedrons';  
    elements = elements(:); % Flatten to (4*N_tet x 1)

    % Repeat each node index 3 times (for x, y, z DOFs)
    nodes_rep = repelem(elements, 3);  

    % Create column indices in global displacement vector (u)
    components = repmat([1; 2; 3], 4 * N_tet, 1);  
    columns = 3 * (nodes_rep - 1) + components;  

    % Row indices in G (sequential for 12 DOFs per tetrahedron)
    rows = (1:12*N_tet)';  

    % Sparse transformation matrix G (maps nodal displacements to element displacements)
    G = sparse(rows, columns, 1, 12*N_tet, 3*N_verts);

    % Compute element displacements
    nodal_displacements = nodal_displacements';
    element_displacements = G * nodal_displacements(:);  % (12*N_tet x 3*N_a)

    % -----------------------------------------------------------
    % 2️⃣ Compute element strains and stresses
    % -----------------------------------------------------------

    % Compute the Block Diagonal Strain-Displacement Matrix (6*N_tet x 12*N_tet)
    % Le_blck = From3DToBlockDiagonalMatrix(Le, true);
    Le_cells = squeeze(num2cell(Le, [1, 2]));  % Convert to a cell array of matrices
    Le_blck = sparse(blkdiag(Le_cells{:}));            % Create block-diagonal matrix

    % Compute element strains (6 * N_tet x 3*N_a*N_tet)
    pred_strains = Le_blck * element_displacements;

    % Compute the Block Diagonal Stiffness Matrix (6*N_tet x 6*N_tet)
    Ce_blck = kron(eye(N_tet),Ce);
    Ce_blck = sparse(Ce_blck);

    % Compute element stresses (6*N_tet x 3*N_a)
    pred_stresses_element = Ce_blck * pred_strains;

    % -----------------------------------------------------------
    % 3️⃣ Construct matrix W: maps element stresses to nodal stresses
    % -----------------------------------------------------------

    % Construct sparse mapping matrix W (6*N_verts x 6*N_tet)
    % Transpose and linearize node indices for rows
    rows = meshTetrahedrons';  % Transpose to 4×N_tet
    rows = rows(:);            % Flatten to 4*N_tet × 1

    % Column indices: [1,1,1,1,2,2,2,2,...,N_tet,N_tet,N_tet,N_tet]
    cols = repelem(1:N_tet, 4)';  % Repeat each tetrahedron index 4 times

    % Basic sparse matrix for node-element mapping (N_verts x N_tet)
    W_basic = sparse(rows, cols, 1, N_verts, N_tet);

    % Expand W to handle 6 stress components per node
    W_blck = kron(W_basic, speye(6)); % (6*N_verts x 6*N_tet)

    % Compute nodal stresses
    pred_nodal_stresses = W_blck * pred_stresses_element; % (6*N_verts x 3*N_a)

    % Normalize by the number of contributions per node
    denom = sum(W_blck, 2); % (6*N_verts x 3*N_a)
    pred_nodal_stresses = pred_nodal_stresses ./ (denom + eps);


    % -----------------------------------------------------------
    % 4️⃣ Compute the constraint matrix A
    % -----------------------------------------------------------

    P3 = reshape(pred_nodal_stresses, [6, N_verts]);  

    % extract components
    s11 = squeeze(P3(1,:));
    s22 = squeeze(P3(2,:));
    s33 = squeeze(P3(3,:));
    s12 = squeeze(P3(4,:));
    s13 = squeeze(P3(5,:));
    s23 = squeeze(P3(6,:));
    
    % von Mises
    % VM = sqrt(0.5*((s11 - s22).^2 + (s22 - s33).^2 + (s33 - s11).^2) + 3*(s12.^2 + s13.^2 + s23.^2));
    den = 2*VM_stress' + eps;
    
    % derivative weights (all N_verts × 3N_a)
    d11 = (2*s11 - s22 - s33) ./ den;
    d22 = (-s11 + 2*s22 - s33) ./ den;
    d33 = (-s11 - s22 + 2*s33) ./ den;
    d12 = (6*s12)           ./ den;
    d13 = (6*s13)           ./ den;
    d23 = (6*s23)           ./ den;
    
    % stack back into same shape as P3
    D3 = zeros(6, N_verts);
    D3(1, :) = d11;
    D3(2, :) = d22;
    D3(3, :) = d33;
    D3(4, :) = d12;
    D3(5, :) = d13;
    D3(6, :) = d23;
    
    % final directional-derivative: dot‐product along the first dimension
    grad3 = squeeze( sum( D3 .* P3, 1 ) );   % now size N_verts × 3*N_a
 
    % -----------------------------------------------------------
    % 5️⃣  Return the final gradient
    % -----------------------------------------------------------
    grad_h = grad3;

end
