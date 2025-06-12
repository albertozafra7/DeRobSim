clc; clear; close all;
CorrectVonMises;
%% ------------------------------------------------------------------------
% PRECONDICIONES (dichas variables ya deben existir en tu workspace):
%   MeshTetrahedrons   : (N_tet × 4) índices de los nodos de cada tetraedro
%   VertexInitPoses    : (n_nodes × 3) coordenadas iniciales (X) de cada nodo
%   VertexFinalPoses   : (n_nodes × 3) coordenadas deformadas (x) de cada nodo
%   E = 210e3; nu = 0.3;        % propiedades del material
%   Ce = CeMatrixComputation(E, nu);      
%     % Ce es la matriz 6×6 isotrópica en Voigt [xx,yy,zz,xy,yz,zx].
%   Le = ComputeBe(MeshTetrahedrons, VertexInitPoses);
%     % Le es 6×12×N_tet, la B‐matrix de cada elemento linear tetraédrico.
%
%   R (struct) = solve(model)    % para comparar errores: R.Mesh, R.Stress, R.VonMisesStress
%
% SALIDA QUE QUEREMOS:
%   - stress_n_disp(:, :, nd) → vector 6×1 (sigma_xx,sigma_yy,sigma_zz,sigma_xy,sigma_yz,sigma_zx) en cada nodo nd
%   - VMSigma_n_Disp(nd)      → von Mises en cada nodo nd
%
% AL FINAL: comparar con 
%   R.Stress.*  y  R.VonMisesStress 
% para ver error absoluto / relativo.

%% 1) PREALOCACIONES
% N_tet   = size(MeshTetrahedrons, 1);
% n_nodes = size(VertexInitPoses,   1);
% 
% Re              = zeros(3,3,N_tet);   % Rotación ortonormal de cada tetra
% strains_local   = zeros(6,1,N_tet);   % Cepas en marco local (6×1 por cada tetra)
% stresses_local  = zeros(6,1,N_tet);   % Tensiones en marco local (6×1 por tetra)
% stresses_global = zeros(6,1,N_tet);   % Tensiones en marco global (6×1 por tetra)
% u_loc_e = zeros(12,1,N_tet);
% 
% [Re,X0c,XFc] = RotationComputation(VertexInitPoses,VertexFinalPoses,MeshTetrahedrons);
% 
% % -------------------------------------------
% % 1️⃣ Obtener las posiciones relativas al centroide
% % (Asumimos que VertexInitPoses, VertexFinalPoses, MeshTetrahedrons, Re, X0c y XFc
% % ya están calculados en el workspace.)
% % -------------------------------------------
% 
% % 1️⃣ Extraer todas las posiciones iniciales y finales en bloque
% allIndices = MeshTetrahedrons';            % 4×N_tet
% allIndices = allIndices(:);                % 4N_tet×1
% 
% X0_all = VertexInitPoses(allIndices, :);   % (4N_tet×3)
% XF_all = VertexFinalPoses(allIndices, :);  % (4N_tet×3)
% 
% % 2️⃣ Reorganizar a (3×4×N_tet)
% X0_all = permute(reshape(X0_all, 4, N_tet, 3), [3,1,2]);  % 3×4×N_tet
% XF_all = permute(reshape(XF_all, 4, N_tet, 3), [3,1,2]);  % 3×4×N_tet
% 
% % 3️⃣ Restar centroides para obtener vectores relativos
% X0_rel = X0_all - reshape(X0c, [3,1,N_tet]);   % 3×4×N_tet
% xF_rel = XF_all - reshape(XFc, [3,1,N_tet]);   % 3×4×N_tet
% 
% % 4️⃣ Calcular la rotación inversa (transpuesta)
% Rinv_all = pagetranspose(Re);   % 3×3×N_tet
% 
% % 4️⃣ Calcular la rotación inversa (transpuesta)
% % Actually, in co-rotational FEM, it’s often recommended to use Re directly
% % to transform the deformed relative position into the reference frame.
% % So here we apply Re' to xF_rel to get local displacements:
% xv_rotRemoved = zeros(3,4,N_tet);
% for v = 1:4
%     % Rotar cada vértice de cada tetraedro en paralelo
%     xv_rotRemoved(:,v,:) = pagemtimes(pagetranspose(Re), xF_rel(:,v,:));  % 3×1×N_tet → 3×1×N_tet
% 
% end
% 
% 
% % 6️⃣ Calcular el desplazamiento local co-rotacional
% uv_loc = xv_rotRemoved - X0_rel;  % 3×4×N_tet
% 
% % 7️⃣ Ensamblar u_loc_e (12×1×N_tet) en el orden correcto vértice-por-vértice
% u_loc_e = zeros(12,1,N_tet);
% for v = 1:4
%     rows = (v-1)*3 + (1:3);
%     u_loc_e(rows,1,:) = uv_loc(:,v,:);
% end
% 
% 
% 
% strains_local = pagemtimes(Le,u_loc_e);
% stresses_local = pagemtimes(Ce,strains_local);
% 
% 
% % 1) Reconstruir, para todos los elementos a la vez, las matrices 3×3 en marco local:
% Sigma_loc_all = zeros(3,3,N_tet);
% Sigma_loc_all(1,1,:) = stresses_local(1,1,:);  % σ_xx
% Sigma_loc_all(2,2,:) = stresses_local(2,1,:);  % σ_yy
% Sigma_loc_all(3,3,:) = stresses_local(3,1,:);  % σ_zz
% Sigma_loc_all(1,2,:) = stresses_local(4,1,:);  % σ_xy
% Sigma_loc_all(2,1,:) = stresses_local(4,1,:);  % σ_xy
% Sigma_loc_all(2,3,:) = stresses_local(5,1,:);  % σ_yz
% Sigma_loc_all(3,2,:) = stresses_local(5,1,:);  % σ_yz
% Sigma_loc_all(3,1,:) = stresses_local(6,1,:);  % σ_zx
% Sigma_loc_all(1,3,:) = stresses_local(6,1,:);  % σ_zx
% 
% % 2) Rotar cada Sigma_loc(:,:,e) a global con pagemtimes:
% Sigma_glob_all = pagemtimes(pagemtimes(Re, Sigma_loc_all), pagetranspose(Re));
% 
% 
% % 3) Extraer en bloque las 6 componentes Voigt de cada Sigma_glob:
% stresses_global = zeros(6,1,N_tet);
% stresses_global(1,1,:) = Sigma_glob_all(1,1,:);  % σ_xx
% stresses_global(2,1,:) = Sigma_glob_all(2,2,:);  % σ_yy
% stresses_global(3,1,:) = Sigma_glob_all(3,3,:);  % σ_zz
% stresses_global(4,1,:) = Sigma_glob_all(1,2,:);  % σ_xy
% stresses_global(5,1,:) = Sigma_glob_all(2,3,:);  % σ_yz
% stresses_global(6,1,:) = Sigma_glob_all(3,1,:);  % σ_zx
% 
% %% 3) PROMEDIAR LAS TENSIONES GLOBALES EN CADA NODO
% stress_n_disp = zeros(6,1,n_nodes);
% node_contrib  = zeros(n_nodes,1);
% 
% for e = 1:N_tet
%     nodes = MeshTetrahedrons(e,:);
%     s_glob = stresses_global(:,:,e);  % 6×1 en global
% 
%     for i = 1:4
%         nd = nodes(i);
%         stress_n_disp(:,:,nd) = stress_n_disp(:,:,nd) + s_glob;
%         node_contrib(nd) = node_contrib(nd) + 1;
%     end
% end
% 
% for nd = 1:n_nodes
%     if node_contrib(nd) > 0
%         stress_n_disp(:,:,nd) = stress_n_disp(:,:,nd) / node_contrib(nd);
%     end
% end
% 
[elem_stress_Rot, nodal_stress_Rot] = ComputeRotationalStresses(VertexInitPoses,VertexFinalPoses,MeshTetrahedrons,Le,Ce);


%% 4) CALCULAR VON MISES EN CADA NODO (CON TENSIONES GLOBALES)
VMSigma_nodal_Rot = zeros(n_nodes,1);
for nd = 1:n_nodes
    s_n  = nodal_stress_Rot(:,:,nd);  % [sigma_xx; sigma_yy; sigma_zz; sigma_xy; sigma_yz; sigma_zx]
    sigma_xx = s_n(1);
    sigma_yy = s_n(2);
    sigma_zz = s_n(3);
    sigma_xy = s_n(4);
    sigma_yz = s_n(5);
    sigma_zx = s_n(6);

    VMSigma_nodal_Rot(nd) = sqrt( ...
         0.5*((sigma_xx - sigma_yy)^2 + (sigma_yy - sigma_zz)^2 + (sigma_zz - sigma_xx)^2) ...
       + 3*( sigma_xy^2 + sigma_yz^2 + sigma_zx^2 ) );
end




%% 5) GRAFICAR Y COMPARAR ERRORES
figure;
pdeviz(R.Mesh, VMSigma_nodal_Rot);
title("von Mises co‐rotacional (SVD + centroide)");

Result_Stress = [R.Stress.sxx, R.Stress.syy, R.Stress.szz, ...
                 R.Stress.sxy, R.Stress.syz, R.Stress.szx];
stressError = abs(squeeze(nodal_stress_Rot)' - Result_Stress);

disp("Máx. error absoluto en tensiones nodales (global):");
disp(max(stressError));

VMError = abs(VMSigma_nodal_Rot - R.VonMisesStress) ./ R.VonMisesStress;
disp("Máx. error relativo von Mises nodal:");
disp(max(VMError));
disp("Máx. % error von Mises nodal:");
disp(100 * max(VMError) + "%");
