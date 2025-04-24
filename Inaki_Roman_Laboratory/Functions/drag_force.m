function FD = drag_force(u)
    % Parámetros de entrada
    Cd = u(1);      % Coeficiente de arrastre
    Af = u(2);      % Área frontal
    vx = u(3);      % Velocidad del vehículo (puede ser negativa)
    Cr = u(4);      % Coeficiente de resistencia a la rodadura
    m = u(5);       % Masa del vehículo
    g = u(6);       % Aceleración gravitacional
    theta = u(7);   % Ángulo de la pendiente de la carretera

    % Constante de densidad del aire
    rho = 1.225; % kg/m^3


    % Calcular el término de arrastre
    drag_term = 0.5 * rho * Cd * Af * vx^2;
    if abs(vx) < 1e-3
        drag_term = 0;  % Evitar valores pequeños que amplifiquen el resultado
    end

    % Calcular la resistencia a la rodadura
    rolling_resistance = Cr * m * g * cos(theta);

    % Fuerza total de arrastre
    FD = drag_term + rolling_resistance;

    % Validar si el resultado es finito
    if ~isfinite(FD)
        FD = 0; 
        warning('FD resultó no válido, asignando FD = 0.');
    end
end