function omega = convertlw(u)
    % Entradas:
    % u(1) = vx: Velocidad lineal del vehículo
    % u(2) = lambda: Slip ratio
    % u(3) = R: Radio de la rueda
    % u(4) = mode: Modo (1 = frenado, 2 = aceleración)

    % Variables de entrada
    fx = u(1);   
    lambda = u(2); 
    R = u(3); 
    mode = u(4); 

    % Threshold for near-zero values
    epsilon = 1e-6;
    if abs(fx) < epsilon
        fx = sign(fx) * epsilon;
    end
    if abs(R) < epsilon
        R = sign(R) * epsilon;
    end

    % Compute omega based on mode
    if mode == 1 % braking
        omega = (lambda * fx + fx) / R;
    elseif mode == 2 % acceleration
        omega = (fx * (lambda + 1)) / R;
    else
        error('Invalid mode. Use 1 for braking or 2 for acceleration.');
    end
end