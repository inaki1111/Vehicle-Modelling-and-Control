function beta_rate = computeBetaRate(u)
    Fyf = u(1);  % Lateral force at the front
    Fyr = u(2);  % Lateral force at the rear
    m = u(3);    % Vehicle mass
    vx = u(4);   % Longitudinal velocity
    yaw_rate = u(5); % Yaw rate

    % Pequeña constante para evitar divisiones por cero o valores problemáticos
    epsilon = 1e-6;

    % Validación de entradas
    if abs(Fyf) < epsilon
        Fyf = sign(Fyf) * epsilon; % Ajustar a un valor mínimo con el mismo signo
    end
    if abs(Fyr) < epsilon
        Fyr = sign(Fyr) * epsilon; % Ajustar a un valor mínimo con el mismo signo
    end
    if abs(m) < epsilon
        m = epsilon; % Evitar división por cero en la masa
    end
    if abs(vx) < epsilon
        vx = epsilon; % Evitar división por cero en la velocidad longitudinal
    end
    if abs(yaw_rate) < epsilon
        yaw_rate = sign(yaw_rate) * epsilon; % Ajustar yaw_rate mínimo con el mismo signo
    end

    % Compute beta_rate
    beta_rate = (Fyf + Fyr) / (m * vx) - yaw_rate;
end