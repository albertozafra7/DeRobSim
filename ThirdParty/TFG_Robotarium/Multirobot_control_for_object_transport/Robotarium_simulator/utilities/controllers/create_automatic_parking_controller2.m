function [ automatic_parking_controller ] = create_automatic_parking_controller2(varargin)
% CREATE_AUTOMATIC_PARKING_CONTROLLER Creates a controller that drive a
% unicycle-modeled sytsem to a particular point and stops it (within
% tolerances)
% Works by driving the unicycle to within PositionError of the point then
% rotating it to within RotationError of the desired rotation
%
%   Args:
%       LinearVelocityGain, optional: see also
%       AngularVelocityLimit, optional: see also
%       PositionError, optional: Error tolerance for position
%       RotationError, optional: Error tolerance for rotation
%       VelocityMagnitudeLimit, optional: Limit for velocity while driving
%       to position
%
% See also CREATE_SI_TO_UNI_MAPPING3

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.5);
    addOptional(p, 'AngularVelocityLimit', pi/2);
    addOptional(p, 'PositionError', 0.03); 
    addOptional(p, 'RotationError', 0.25);
    addOptional(p, 'VelocityMagnitudeLimit', 0.2)
    parse(p, varargin{:});
    
    lin_vel_gain = p.Results.LinearVelocityGain; 
    ang_vel_gain = p.Results.AngularVelocityLimit;
    vel_mag_limit = p.Results.VelocityMagnitudeLimit;
    pos_err = p.Results.PositionError;
    rot_err = p.Results.RotationError;
    
    position_controller = create_si_to_uni_dynamics('LinearVelocityGain', lin_vel_gain, ...
    'AngularVelocityLimit', ang_vel_gain);

    automatic_parking_controller = @automatic_parking_controller_;

    function dxu = automatic_parking_controller_(states, poses)
        
        N = size(states, 2);    % Número de robots
        dxu = zeros(2, N);      % Crea matriz de 0 de 2 filas (x,y) y N columnas
        
        for i = 1:N
            
            wrapped = poses(3, i) - states(3, i);           % Diferencia entre el ángulo deseado y el actual
            wrapped = atan2(sin(wrapped), cos(wrapped));    % Expresa el ángulo en un rango +- pi
            
            dxi = poses(1:2, i) - states(1:2, i);           % Diferencia entre posición deseada y actual
            
            % Normalize 
            norm_ = norm(dxi);
            if(norm_ > vel_mag_limit)
               dxi = vel_mag_limit*dxi/norm_;   % Obtiene la distancia máxima a la que se puede llegar con la velocidad límite
            end
            
            % Una vez obtenida la distancia máxima, se lleva el robot a la
            % posición correcta.
            if(norm(dxi) > pos_err)                                     % Si la distancia es mayor al error permitido, se usa el controlador.
                dxu(:, i) = position_controller(dxi, states(:, i));
           % Cuando el robot alcanza la posición, se controla el ángulo.
            elseif(abs(wrapped) > rot_err)                              % Si el ángulo es mayor al error permitido, la vel. lineal será 0 (porque ya ha llegado a la posición correcta),
                dxu(1, i) = 0;                                          % pero la vel. angular se modifica en la matriz dxu.
                dxu(2, i) = wrapped;
            else
                dxu(:, i) = zeros(2, 1);        % Cuando el robot esté en la posición y ángulo correctos, los términos correspondientes de la matriz dxu se anulan para que el robot deje de moverse.
            end            
        end       
    end    
end


