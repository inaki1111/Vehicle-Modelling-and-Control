function alpha = computeSlipAngle(u)
    vx = u(1);       % Longitudinal velocity
    beta = u(2);     % Side-slip angle
    yaw_rate = u(3); % Yaw rate
    l = u(4);        % Distance to axle
    delta = u(5);    % Steering angle

    % Ensure nonzero values while preserving signs
    safe_vx = vx + (vx == 0) * 1e-6 * sign(vx + eps);
    safe_beta = beta + (beta == 0) * 1e-6 * sign(beta + eps);
    safe_yaw_rate = yaw_rate + (yaw_rate == 0) * 1e-6 * sign(yaw_rate + eps);

    % Compute slip angle
    alpha = delta - atan((safe_vx * safe_beta + l * safe_yaw_rate) / safe_vx);
end