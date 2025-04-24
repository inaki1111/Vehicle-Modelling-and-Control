
yalmip('clear');
global controller
% Parámetros del vehículo y del MPC
m = 1400;
Iz = 1960;
lf = 1.177;
lr = 1.358;
Cf = 84085;
Cr = 87342;
Ts = 0.1;
N = 10;
vx = 10;
nx = 2;
nu = 1;

A_cont = [ -(Cf+Cr)/(m*vx),       ((Cr*lr - Cf*lf)/(m*vx) - vx);
           (Cr*lr - Cf*lf)/(Iz*vx), -(lf^2*Cf + lr^2*Cr)/(Iz*vx)];

B_cont = [ Cf/m;
           (lf*Cf)/Iz ];
Ad = eye(nx) + A_cont*Ts;
Bd = B_cont*Ts;

Q = diag([0.1, 100]);  
R = 1;

umin = -5;
umax = 5;
xmin = [-1; -1];
xmax = [1; 1];

psi_dot_ref = 0.15;
xref = [0; psi_dot_ref];
x0 = [0.1; 0.1];

x = sdpvar(nx, N+1);
u = sdpvar(nu, N);

constraints = x(:,1) == x0;
objective = 0;
for k = 1:N
    constraints = [constraints, x(:,k+1) == Ad*x(:,k) + Bd*u(:,k)];
    constraints = [constraints, umin <= u(:,k) <= umax];
    constraints = [constraints, xmin <= x(:,k+1) <= xmax];
    objective = objective + (x(:,k)-xref)'*Q*(x(:,k)-xref) + u(:,k)'*R*u(:,k);
end
objective = objective + (x(:,N+1)-xref)'*Q*(x(:,N+1)-xref);

% Crear el objeto controller (tipo optimizer)
controller = optimizer(constraints, objective, sdpsettings('solver','quadprog','verbose',0), x(:,1), u(:,1));

% Hasta aquí definimos el objeto 'controller' en el workspace de MATLAB.