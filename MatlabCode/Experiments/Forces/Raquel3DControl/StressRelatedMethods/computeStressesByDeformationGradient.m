function [stresses_GL, volumes] = computeStressesByDeformationGradient(...
    MeshTetrahedrons, VertexInitPoses, VertexCurrentPoses, E, nu)

  % Precompute elasticity matrix
  Ce = CeMatrixComputation(E, nu);    % 6×6

  N_tet = size(MeshTetrahedrons,1);
  stresses_GL = zeros(6, N_tet);
  volumes     = zeros(1, N_tet);

  for e = 1:N_tet
    idx = MeshTetrahedrons(e,:);

    % Reference positions
    X0 = VertexInitPoses(idx(1),:);
    X1 = VertexInitPoses(idx(2),:);
    X2 = VertexInitPoses(idx(3),:);
    X3 = VertexInitPoses(idx(4),:);

    % Current positions
    x0 = VertexCurrentPoses(idx(1),:);
    x1 = VertexCurrentPoses(idx(2),:);
    x2 = VertexCurrentPoses(idx(3),:);
    x3 = VertexCurrentPoses(idx(4),:);

    % 1) Build shape matrices
    Dm = [ (X1-X0)' , (X2-X0)' , (X3-X0)' ];  % 3×3
    Ds = [ (x1-x0)' , (x2-x0)' , (x3-x0)' ];  % 3×3

    % 2) Compute volume = |det(Dm)|/6
    detDm = det(Dm);
    volumes(e) = abs(detDm)/6;
    if abs(detDm)<1e-12
      error('Inverted or degenerate element %d', e);
    end

    % 3) Deformation gradient
    F = Ds / Dm;  % same as Ds * inv(Dm)

    % 4) Green–Lagrange strain tensor
    E_tensor = 0.5*(F'*F - eye(3));

    % 5) Voigt vector (engineering shear)
    eps_voigt = [ E_tensor(1,1);
                  E_tensor(2,2);
                  E_tensor(3,3);
             2*E_tensor(1,2);
             2*E_tensor(2,3);
             2*E_tensor(1,3) ];

    % 6) Stress in Voigt form
    sigma_voigt = Ce * eps_voigt;  % 6×1

    stresses_GL(:,e) = sigma_voigt;
  end
end
% 
