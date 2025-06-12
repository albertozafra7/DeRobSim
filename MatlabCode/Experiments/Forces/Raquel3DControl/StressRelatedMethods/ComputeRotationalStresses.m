function [stresses_global, stress_n_disp] = ComputeRotationalStresses(VertexInitPoses,VertexFinalPoses,MeshTetrahedrons,Le,Ce)
N_tet   = size(MeshTetrahedrons, 1);
n_nodes = size(VertexInitPoses,   1);

[Re,X0c,XFc] = RotationComputation(VertexInitPoses,VertexFinalPoses,MeshTetrahedrons);

% Posiciones iniciales y finales en bloque
allIndices = MeshTetrahedrons';            % 4×N_tet
allIndices = allIndices(:);                % 4N_tet×1

X0_all = VertexInitPoses(allIndices, :);   % (4N_tet×3)
XF_all = VertexFinalPoses(allIndices, :);  % (4N_tet×3)

% 2️⃣ Reorganizar a (3×4×N_tet)
X0_all = permute(reshape(X0_all, 4, N_tet, 3), [3,1,2]);  % 3×4×N_tet
XF_all = permute(reshape(XF_all, 4, N_tet, 3), [3,1,2]);  % 3×4×N_tet

% 3️⃣ Restar centroides para obtener vectores relativos
X0_rel = X0_all - reshape(X0c, [3,1,N_tet]);   % 3×4×N_tet
xF_rel = XF_all - reshape(XFc, [3,1,N_tet]);   % 3×4×N_tet

% 4️⃣ Calcular la rotación inversa (transpuesta)
Rinv_all = pagetranspose(Re);   % 3×3×N_tet

% 4️⃣ Calcular la rotación inversa (transpuesta)
% Actually, in co-rotational FEM, it’s often recommended to use Re directly
% to transform the deformed relative position into the reference frame.
% So here we apply Re' to xF_rel to get local displacements:
xv_rotRemoved = zeros(3,4,N_tet);
for v = 1:4
    % Rotar cada vértice de cada tetraedro en paralelo
    xv_rotRemoved(:,v,:) = pagemtimes(pagetranspose(Re), xF_rel(:,v,:));  % 3×1×N_tet → 3×1×N_tet
    
end


% 6️⃣ Calcular el desplazamiento local co-rotacional
uv_loc = xv_rotRemoved - X0_rel;  % 3×4×N_tet

% 7️⃣ Ensamblar u_loc_e (12×1×N_tet) en el orden correcto vértice-por-vértice
u_loc_e = zeros(12,1,N_tet);
for v = 1:4
    rows = (v-1)*3 + (1:3);
    u_loc_e(rows,1,:) = uv_loc(:,v,:);
end



strains_local = pagemtimes(Le,u_loc_e);
stresses_local = pagemtimes(Ce,strains_local);


% 1) Reconstruir, para todos los elementos a la vez, las matrices 3×3 en marco local:
Sigma_loc_all = zeros(3,3,N_tet);
Sigma_loc_all(1,1,:) = stresses_local(1,1,:);  % σ_xx
Sigma_loc_all(2,2,:) = stresses_local(2,1,:);  % σ_yy
Sigma_loc_all(3,3,:) = stresses_local(3,1,:);  % σ_zz
Sigma_loc_all(1,2,:) = stresses_local(4,1,:);  % σ_xy
Sigma_loc_all(2,1,:) = stresses_local(4,1,:);  % σ_xy
Sigma_loc_all(2,3,:) = stresses_local(5,1,:);  % σ_yz
Sigma_loc_all(3,2,:) = stresses_local(5,1,:);  % σ_yz
Sigma_loc_all(3,1,:) = stresses_local(6,1,:);  % σ_zx
Sigma_loc_all(1,3,:) = stresses_local(6,1,:);  % σ_zx

% 2) Rotar cada Sigma_loc(:,:,e) a global con pagemtimes:
Sigma_glob_all = pagemtimes(pagemtimes(Re, Sigma_loc_all), pagetranspose(Re));


% 3) Extraer en bloque las 6 componentes Voigt de cada Sigma_glob:
stresses_global = zeros(6,1,N_tet);
stresses_global(1,1,:) = Sigma_glob_all(1,1,:);  % σ_xx
stresses_global(2,1,:) = Sigma_glob_all(2,2,:);  % σ_yy
stresses_global(3,1,:) = Sigma_glob_all(3,3,:);  % σ_zz
stresses_global(4,1,:) = Sigma_glob_all(1,2,:);  % σ_xy
stresses_global(5,1,:) = Sigma_glob_all(2,3,:);  % σ_yz
stresses_global(6,1,:) = Sigma_glob_all(3,1,:);  % σ_zx

%% 3) PROMEDIAR LAS TENSIONES GLOBALES EN CADA NODO
stress_n_disp = zeros(6,1,n_nodes);
node_contrib  = zeros(n_nodes,1);

for e = 1:N_tet
    nodes = MeshTetrahedrons(e,:);
    s_glob = stresses_global(:,:,e);  % 6×1 en global
    
    for i = 1:4
        nd = nodes(i);
        stress_n_disp(:,:,nd) = stress_n_disp(:,:,nd) + s_glob;
        node_contrib(nd) = node_contrib(nd) + 1;
    end
end

for nd = 1:n_nodes
    if node_contrib(nd) > 0
        stress_n_disp(:,:,nd) = stress_n_disp(:,:,nd) / node_contrib(nd);
    end
end

end