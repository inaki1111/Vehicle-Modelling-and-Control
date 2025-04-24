function xy_dot = computeVehiclePosition(u)

    vx = u(1); % Longitudinal velocity
    beta = u(2); % Side-slip angle
    psi = u(3); % Yaw angle

 
    vy = vx * tan(beta);
    x_dot = vx * cos(psi) - vy * sin(psi); 
    y_dot = vx * sin(psi) + vy * cos(psi); 

    xy_dot = [x_dot; y_dot];
end