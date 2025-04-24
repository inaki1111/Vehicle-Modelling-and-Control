function y = lateral_force(u)
    % Input variables
    vx = u(1);          % Longitudinal velocity
    beta = u(2);        % Side-slip angle
    yaw_rate = u(3);    % Yaw rate
    lf = u(4);          % Distance to axle front
    lr = u(5);          % Distance to axle rear
    steering_angle = u(6); % Steering angle
    lambda = u(7);      % Slip ratio
    mu = u(8);          % Friction coefficient

    % Small constant to avoid zero initialization
    epsilon = 1e-6;

    % Ensure non-zero values for critical variables
    if abs(vx) < epsilon
        vx = sign(vx) * epsilon; % Small non-zero vx
    end
    if abs(beta) < epsilon
        beta = sign(beta) * epsilon; % Small non-zero beta
    end
    if abs(yaw_rate) < epsilon
        yaw_rate = sign(yaw_rate) * epsilon; % Small non-zero yaw_rate
    end
    if abs(lambda) < epsilon
        lambda = sign(lambda) * epsilon; % Small non-zero lambda
    end

    % Compute Pacejka parameters
    By = 2 * (2 - mu) * 8.3278;
    Cy = (5 / 4 - mu / 4) * 1.1009;
    Dy = mu * 2268;
    Ey = -1.1661;

    % Compute slip angles
    alpha_f = steering_angle - atan((vx * beta + lf * yaw_rate) / (vx + epsilon));
    alpha_r = -atan((vx * beta - lr * yaw_rate) / (vx + epsilon));

    % Compute lateral forces
    % Front wheels
    Fy_front = Dy * sin(Cy * (By * (1 - Ey) * alpha_f + Ey * atan(By * alpha_f))) * exp(-6 * abs(lambda)^5);
    if abs(Fy_front) < epsilon
        Fy_front = sign(Fy_front) * epsilon; % Avoid zero forces
    end
    Fyf = 2 * Fy_front;

    % Rear wheels
    Fy_rear = Dy * sin(Cy * (By * (1 - Ey) * alpha_r + Ey * atan(By * alpha_r))) * exp(-6 * abs(lambda)^5);
    if abs(Fy_rear) < epsilon
        Fy_rear = sign(Fy_rear) * epsilon; % Avoid zero forces
    end
    Fyr = 2 * Fy_rear;

    % Output of the function
    y = [Fyf Fyr];
end