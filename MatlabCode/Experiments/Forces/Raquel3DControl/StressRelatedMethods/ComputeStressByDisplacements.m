function [stress_elem, stress_nodal, strain_elem, strain_nodal, u_hat_e] = ComputeStressByDisplacements(nodal_disp,MeshTetrahedrons, Ce, Le)

    n_nodes = size(nodal_disp,1);
    % Initialize displacement vector (12x1xN_tet)
    u_hat_e = zeros(12,1,size(MeshTetrahedrons, 1));
    
    % Construct displacement vectors for all tetrahedrons
    for elem = 1:size(MeshTetrahedrons, 1)
        nodes = MeshTetrahedrons(elem, :); % Get the global node indices
        for node = 1:4
            u_hat_e(3*node-2,1,elem) = nodal_disp(nodes(node),1); % x-displacement
            u_hat_e(3*node-1,1,elem) = nodal_disp(nodes(node),2); % y-displacement
            u_hat_e(3*node,1,elem)   = nodal_disp(nodes(node),3); % z-displacement
        end
    end
    
    % Compute strains (6x1xN_tet)
    strain_elem = pagemtimes(Le, u_hat_e);
    
    % Compute stresses (6x1xN_tet)
    stress_elem = pagemtimes(Ce, strain_elem);
    
    % Initialize node-based stress tensor
    stress_nodal = zeros(6,1,n_nodes);
    strain_nodal = zeros(6,1,n_nodes);
    node_contributions = zeros(n_nodes, 1); % Track contributions per node
    
    % Accumulate stresses for each node
    for elem = 1:size(MeshTetrahedrons, 1)
        nodes = MeshTetrahedrons(elem, :);
        elem_stress = stress_elem(:,:,elem);
        elem_strain = strain_elem(:,:,elem);
        for i = 1:4
            stress_nodal(:,:,nodes(i)) = stress_nodal(:,:,nodes(i)) + elem_stress;
            strain_nodal(:,:,nodes(i)) = strain_nodal(:,:,nodes(i)) + elem_strain;
            node_contributions(nodes(i)) = node_contributions(nodes(i)) + 1;
        end
    end
    
    % Average stress at each node
    for node = 1:n_nodes
        if node_contributions(node) > 0
            stress_nodal(:,:,node) = stress_nodal(:,:,node) / node_contributions(node);
            strain_nodal(:,:,node) = strain_nodal(:,:,node) / node_contributions(node);
        end
    end

    strain_elem = squeeze(strain_elem); % (6xN_tet)
    stress_elem = squeeze(stress_elem); % (6xN_tet)
    strain_nodal = squeeze(strain_nodal); % (6xN_verts)
    stress_nodal = squeeze(stress_nodal); % (6xN_verts)

end