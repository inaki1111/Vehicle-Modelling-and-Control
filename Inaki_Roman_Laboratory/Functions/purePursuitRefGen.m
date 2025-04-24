function Ref = purePursuitRefGen(u)
    % Declarar posición y orientación inicial del robot
    persistent initialized Xinit Yinit YawInit pathindex;
    global track_x track_y % Trayectoria global
    
    if isempty(initialized)
        % Inicializar variables persistentes
        Xinit = 0; 
        Yinit = 0; 
        YawInit = 0; % Inicialización correcta del yaw
        pathindex = findMinDist(Xinit, Yinit, track_x, track_y, 1);
        initialized = true;
    end

    % Extraer datos de entrada
    if isempty(initialized) || initialized
        % Usar condiciones iniciales si es la primera vez
        Xgl = Xinit;
        Ygl = Yinit;
        YawHead = YawInit;
        initialized = false; % Solo usar las condiciones iniciales en la primera llamada
    else
        % Actualizar estado con las entradas si ya se ha inicializado
        Xgl = u(1); % Posición X actual del robot
        Ygl = u(2); % Posición Y actual del robot
        YawHead = u(3); % Orientación actual del robot
    end

    % Extraer velocidad y otros parámetros de entrada
    Vx = u(4); % Velocidad lineal
    Beta = u(5); % Ángulo de deslizamiento lateral
    tp = u(6); % Tiempo de lookahead

    % Imprimir la posición y orientación del robot
    fprintf('Robot Position: X = %.3f, Y = %.3f, Yaw = %.3f\n', Xgl, Ygl, YawHead);

    % Extraer trayectoria global
    Xpath = track_x;
    Ypath = track_y;

    % Verificar velocidad mínima
    if abs(Vx) < 0.05
        Vx = 0.05;
    end
    Vy = Vx * Beta; % Velocidad lateral
    V = sqrt(Vx^2 + Vy^2);
    L = tp * V; % Distancia de lookahead

    % Encontrar el nuevo índice de la trayectoria
    pathindex = findMinDist(Xgl, Ygl, Xpath, Ypath, pathindex);

    % Punto de seguimiento
    XL = Xgl + L * cos(YawHead + Beta); % Posición X a lookahead distance
    YL = Ygl + L * sin(YawHead + Beta); % Posición Y a lookahead distance
    pursuitindex = findMinDist(XL, YL, Xpath, Ypath, pathindex);
    Xpursuit = Xpath(pursuitindex);
    Ypursuit = Ypath(pursuitindex);

    % Calcular yawrate de referencia
    alpha = atan2(Ypursuit - Ygl, Xpursuit - Xgl) - YawHead;
    k = 2 * sin(alpha) / norm([Xgl, Ygl] - [Xpursuit, Ypursuit]); % k = 2*sin(alpha)/L
    Yawrate_Ref = k * Vx;

    % Calcular error lateral
    sgn = sign(sin(alpha));
    ye = sgn * norm([XL, YL] - [Xpursuit, Ypursuit]);

    % Salida de referencias
    Ref = [Yawrate_Ref; ye];
end

function y = findMinDist(X, Y, Xpath, Ypath, index)
    % Encontrar la distancia mínima desde un punto hasta la trayectoria
    DO = true;
    while DO
        di = norm([X, Y] - [Xpath(index), Ypath(index)]);
        index = index + 1;
        dnew = norm([X, Y] - [Xpath(index), Ypath(index)]);
        DO = di > dnew;
    end
    y = index - 1;
end