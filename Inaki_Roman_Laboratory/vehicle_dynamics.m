
global track_x track_y omega tp m Cf Iz lf lr Cd Af Cr R C_lambda C_alpha_f C_alpha_r lambda g theta mode FN mu1 mu2 mu3 vx miu F_max steering_angle;
m = 1400;        % Masa del vehículo (kg)
Iz = 1960;       % Inercia del vehículo respecto al eje vertical (kg·m^2)
lf = 1.177;      % Distancia al eje delantero (m)
lr = 1.358;      % Distancia al eje trasero (m)
Cd = 0.35;       % Coeficiente de arrastre aerodinámico
Af = 2.14;       % Área frontal (m^2)
Cf = 0.015;
Cr = 0.015;      % Coeficiente de resistencia a la rodadura
R = 0.3;         % Radio de las ruedas (m)
C_lambda = 66100;    % Rigidez longitudinal del neumático (N)
C_alpha_f = 84085;   % Rigidez lateral del neumático delantero (N/rad)
C_alpha_r = 87342;   % Rigidez lateral del neumático trasero (N/rad)
omega  = 15;
lambda = 0.1; 
g = 9.81;
theta = 1;
mode = 2;
FN = (m*g)/2;
track = load('data/Track.mat');
track_x = track.Track.X;
track_y = track.Track.Y;

steering_angle = 0.1;

mu1 = 1.11;  % seco 1.11, 23.99 , 0.52 % mojado 0.687 33.822 0.347 cobblestone 1.37, 6.46, 0.67  hielo 0.19 94.13 0.06
mu2 = 23.99;  
mu3 = 0.52;

vx = 10;
tp = 3;
miu = 0.5;

F_max = miu * FN;

% plant 

P = tf(1, [m, 0]); % P(s) = 1 / (m * s)


% controller 

s = tf('s'); 
Kp = 1;      
Ki = 1;   

%C = Kp + Ki / s;


%% MPC

% Matrices del modelo continuo
A = [
    (-Cf - Cr) / (m * vx),   (-vx + (Cr * lr - Cf * lf) / (m * vx));
    (-lf * Cf + lr * Cr) / (Iz * vx),  (-lf^2 * Cf - lr^2 * Cr) / (Iz * vx)
];

B = [
    Cf / m;
    lf * Cf / Iz
];

C = [0 1]; % Solo rastrear la velocidad angular de guiñada (ψ̇)
D = 0;


Ts = 0.1; % Tiempo de muestreo (s)

% Discretización
sys_c = ss(A, B, C, D);       % Sistema en tiempo continuo
sys_d = c2d(sys_c, Ts);       % Sistema discretizado

Adis = sys_d.A;                 % Matriz A discreta
Bdis = sys_d.B;                 % Matriz B discreta
Cdis = sys_d.C;                 % Matriz C discreta
Ddis = sys_d.D;                 % Matriz D discreta

plant = ss(Adis,Bdis,Cdis,Ddis)


%% track


% Ángulo de rotación (en radianes)
theta = -4.17;

% Matriz de rotación
R = [cos(theta), -sin(theta); 
     sin(theta),  cos(theta)];

% Rotar todos los puntos
rotated_points = R * [track_x; track_y];

% Nuevos vectores rotados
track_x_rotated = rotated_points(1, :);
track_y_rotated = rotated_points(2, :);

% Salida
track_x = track_x_rotated;
track_y = track_y_rotated;

figure;
plot(track_x, track_y, '-o'); % Pista rotada
title('Pista Rotada');
xlabel('X');
ylabel('Y');
axis equal;