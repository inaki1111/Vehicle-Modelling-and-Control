% Pose inicial del robot
Xinit = 0;    % Posición inicial X
Yinit = 0;    % Posición inicial Y
YawInit = 4.17; % Orientación inicial en radianes

% Crear el estado inicial del robot
u = [Xinit, Yinit, YawInit, 1, 0, 1]; % [X, Y, Yaw, Vx, Beta, tp]

% Llamar a la función para inicializar
Ref = purePursuitRefGen(u);

% Graficar el track y la pose inicial
figure;
plot(track_x, track_y, 'b-', 'LineWidth', 1.5); % Trayectoria global
hold on;
quiver(Xinit, Yinit, cos(YawInit), sin(YawInit), 2, 'r', 'LineWidth', 4, 'MaxHeadSize', 5); % Pose inicial
legend('Trayectoria', 'Pose inicial');
title('Trayectoria y Pose Inicial del Robot');
xlabel('X [m]');
ylabel('Y [m]');
axis equal;
grid on;