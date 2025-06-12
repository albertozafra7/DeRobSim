function [Re, X0c, XFc] = RotationComputation(...
                         VertexInitPoses, VertexFinalPoses, MeshTetrahedrons)
% RotationComputation  → Re(:,:,e) es la rotación 3×3 ortonormal de cada tetraedro e,
%                       devuelve además los centroides iniciales y deformados (3×N_tet)
%
% Entradas:
%   • VertexInitPoses  (n_nodes×3)
%   • VertexFinalPoses (n_nodes×3)
%   • MeshTetrahedrons (N_tet×4)
%
% Salidas:
%   • Re  : 3×3×N_tet, rotación ortonormal para cada tetra
%   • X0c : 3×N_tet,  centroides iniciales
%   • XFc : 3×N_tet,  centroides deformados

    N_tet   = size(MeshTetrahedrons,1);
    n_nodes = size(VertexInitPoses,1);

    %------------------------------------------------------------------------
    % 1) Creamos una matriz dispersa S (4N_tet × n_nodes) que “seleccione”  
    %    cada vértice de cada tetraedro en el orden correcto. 
    %    Para cada tetra e (1..N_tet) y cada vértice v (1..4), 
    %    hay una fila i = 4*(e−1)+v en S, y una única columna j = MeshTetrahedrons(e,v).
    %    Entonces S(i, j) = 1, y el resto ceros. 
    %
    %    De modo que S * VertexInitPoses = X0_all de tamaño (4N_tet × 3), sin reshape.
    %------------------------------------------------------------------------
    rowsSel = reshape( (1:4*N_tet), 4, N_tet ).';   % N_tet×4 con filas [4(e-1)+1, 4(e-1)+2, … 4(e-1)+4]
    colsSel = MeshTetrahedrons;                     % N_tet×4 (vértices para cada tetra)
    rowsSel = rowsSel(:);                           % 4N_tet×1
    colsSel = colsSel(:);                           % 4N_tet×1
    vals    = ones(4*N_tet,1);

    S = sparse(rowsSel, colsSel, vals, 4*N_tet, n_nodes);
    % Ahora:
    %    X0_all = S * VertexInitPoses;   % → (4N_tet × 3)
    %    XF_all = S * VertexFinalPoses;  % → (4N_tet × 3)

    X0_all = S * VertexInitPoses;   % (4N_tet × 3)
    XF_all = S * VertexFinalPoses;  % (4N_tet × 3)

    %------------------------------------------------------------------------
    % 2) Calcular centroides sin reshape: queremos promediar cada bloque de 4 filas 
    %    en X0_all, es decir, formar X0c(e,:) = (X0_all(4e−3,:)+…+X0_all(4e,:))/4.
    %
    %    Si definimos C = (I_N_tet ⊗ [1/4, 1/4, 1/4, 1/4]) de tamaño (N_tet × 4N_tet),
    %    entonces X0c = C * X0_all  da un resultado (N_tet×3). Al trasponer queda (3×N_tet).
    %------------------------------------------------------------------------
    % Construimos C en forma dispersa:
    block4 = (1/4)*ones(1,4);            % fila [1/4 1/4 1/4 1/4]
    I_et  = speye(N_tet);                % N_tet×N_tet
    C = kron(I_et, block4);              % (N_tet×4N_tet)

    X0c_temp = C * X0_all;               % (N_tet×3)
    XFc_temp = C * XF_all;               % (N_tet×3)

    X0c = X0c_temp.';    % 3×N_tet
    XFc = XFc_temp.';    % 3×N_tet

    %------------------------------------------------------------------------
    % 3) Queremos los vectores relativos  X0_rel_all  = X0_all − repetición_de_X0c 
    %    en bloque sin reshape. Sabemos que X0c_temp es (N_tet×3), mientras que
    %    X0_all es (4N_tet×3). Para “restar el centroide a cada bloque de 4 filas”,
    %    definimos una matriz D (4N_tet×N_tet) = [ I_N_tet; I_N_tet; I_N_tet; I_N_tet ] 
    %    de modo que D * X0c_temp  repite cada centroide 4 veces.
    %------------------------------------------------------------------------
    D = kron( ones(4,1), speye(N_tet) );   % (4N_tet×N_tet)
    X0_rel_all = X0_all - D * X0c_temp;     % 4N_tet×3
    xF_rel_all = XF_all - D * XFc_temp;     % 4N_tet×3

    %------------------------------------------------------------------------
    % 4) Ahora, de 4N_tet×3 queremos extraer, para cada tetra, los primeros 3 vértices 
    %    relativos en bloque. Definimos una matriz Q (3N_tet×4N_tet) que elige filas 
    %    [4(e−1)+1, 4(e−1)+2, 4(e−1)+3] para cada e. Con Q * X0_rel_all obtenemos 
    %    “las 3 aristas iniciales” en bloque como 3N_tet×3.
    %------------------------------------------------------------------------
    rowsQ = reshape( (1:3*N_tet), 3, N_tet ).';  % N_tet×3 → [ (e−1)*3+1, (e−1)*3+2, (e−1)*3+3 ]
    colsQ = reshape( (1:4*N_tet), 4, N_tet ).';  % N_tet×4 → [ 4(e−1)+1, 4(e−1)+2, 4(e−1)+3, 4(e−1)+4 ]
    % Pero solo queremos las tres primeras columnas por elemento:
    rows_list = rowsQ(:);      % 3N_tet×1
    cols_list = zeros(3*N_tet,1);
    for e=1:N_tet
        base4 = 4*(e-1);
        base3 = 3*(e-1);
        cols_list( base3 + (1:3) ) = base4 + (1:3);  
        % para e=1: columnas [1 2 3], e=2: columnas [5 6 7], etc.
    end
    valsQ = ones(3*N_tet,1);
    Q = sparse(rows_list, cols_list, valsQ, 3*N_tet, 4*N_tet);

    E0_big = Q * X0_rel_all;  % (3N_tet × 3)
    EF_big = Q * xF_rel_all;  % (3N_tet × 3)

    %------------------------------------------------------------------------
    % 5) Ahora reorganizamos esos 3N_tet×3 en bloques 3×3×N_tet sin reshape:
    %    Sabemos que E0_big separa en bloques de 3 filas consecutivas para cada e.
    %    Construimos otro kron para indexar cada bloque “3×3” en un solo vector.
    %
    %    Si hacemos: E0_mat = blkdiag( E0_e1, E0_e2, …, E0_eN ) —  que es (3N_tet×3N_tet),
    %    y EF_mat = blkdiag( EF_e1, EF_e2, … ), podemos escribir:
    %       J_mat = EF_mat * inv(E0_mat)
    %    Pero en realidad solo queremos invertir cada 3×3 por separado, 
    %    y no hacer un bloque diagonal completo (que sería muy costoso).
    %
    %    La forma “kron-free” es: para invertir cada bloque 3×3, 
    %    podemos usar la fórmula analítica en bloque, exactamente como antes,
    %    pero operando sobre el arreglo E0_big(3N_tet×3) de tres filas consecutivas.
    %------------------------------------------------------------------------
    % Extraemos en la forma 3×3×N_tet usando indexing “implícito”:
    E0 = zeros(3,3,N_tet);
    EF = zeros(3,3,N_tet);
    for e = 1:N_tet
        rows3 = (3*(e-1)+1) : (3*(e-1)+3);
        E0(:,:,e) = E0_big(rows3, :);
        EF(:,:,e) = EF_big(rows3, :);
    end

    % 6) Invertir E0(:,:,e) por fórmula analítica (sin reshape extra)
    detM = zeros(N_tet,1);
    invE0 = zeros(3,3,N_tet);
    for e = 1:N_tet
        Mloc = E0(:,:,e);
        m11=Mloc(1,1); m12=Mloc(1,2); m13=Mloc(1,3);
        m21=Mloc(2,1); m22=Mloc(2,2); m23=Mloc(2,3);
        m31=Mloc(3,1); m32=Mloc(3,2); m33=Mloc(3,3);
        d = m11*(m22*m33 - m23*m32) ...
          - m12*(m21*m33 - m23*m31) ...
          + m13*(m21*m32 - m22*m31);
        detM(e) = d;
        adj = [  (m22*m33 - m23*m32),  -(m12*m33 - m13*m32),   (m12*m23 - m13*m22);
                -(m21*m33 - m23*m31),   (m11*m33 - m13*m31),  -(m11*m23 - m13*m21);
                 (m21*m32 - m22*m31),  -(m11*m32 - m12*m31),   (m11*m22 - m12*m21) ];
        invE0(:,:,e) = (1/d) * adj;
    end

    % 7) Calcular J(:,:,e) = EF(:,:,e) * invE0(:,:,e)
    J = zeros(3,3,N_tet);
    for e = 1:N_tet
        J(:,:,e) = EF(:,:,e) * invE0(:,:,e);
    end

    % 8) Polar decomposition (SVD) para extraer rotación Re(:,:,e)
    Re = zeros(3,3,N_tet);
    parfor e = 1:N_tet
        [U,~,V] = svd(J(:,:,e));
        Rm = U * V';
        if det(Rm) < 0
            U(:,3) = -U(:,3);
            Rm    = U * V';
        end
        Re(:,:,e) = Rm;
    end
end
