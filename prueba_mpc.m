clc; close all;
yalmip('clear');

m = 1400;
Iz = 1960;
lf = 1.177;
lr = 1.358;
Cf = 84085;
Cr = 87342;
Ts = 0.1;
N = 3;
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

controller = optimizer(constraints, objective, sdpsettings('solver','quadprog','verbose',0), x(:,1), u(:,1));

Tsim = 10;
steps = Tsim/Ts;
x_sim = x0;
u_sim = [];
time = 0:Ts:Tsim;

figure; clf;
subplot(1,2,1); hold on; grid on;
xlabel('time (s)');
ylabel('Steering angle (rad)');
title('Control Input');
plot(time, zeros(size(time)), 'k--');

subplot(1,2,2); hold on; grid on;
xlabel('time (s)');
ylabel('Yaw Rate (rad/s)');
title('Yaw Rate vs Reference');
plot(time, psi_dot_ref*ones(size(time)), 'k--');

for i = 1:steps
    [u_opt, error_flag] = controller{x_sim(:, end)};
    if error_flag ~= 0
        u_opt = 0;
    end
    x_next = Ad*x_sim(:, end) + Bd*u_opt;
    x_sim = [x_sim x_next];
    u_sim = [u_sim u_opt];
    subplot(1,2,1);
    stairs(time(i), u_opt, 'r.', 'MarkerSize', 10);
    subplot(1,2,2);
    plot(time(i), x_sim(2, end), 'bo', 'MarkerSize', 5);
    pause(0.01);
end

subplot(1,2,1);
stairs(time(1:end-1), u_sim, 'r-', 'LineWidth', 1.5);
legend('Steering Input','Location','best');

subplot(1,2,2);
plot(time, x_sim(2, :), 'b-', 'LineWidth', 1.5);
legend('Yaw Rate Reference','Yaw Rate','Location','best');