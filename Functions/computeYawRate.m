function yaw_rate = computeYawRate(u)
    % Input variables
    Fyf = u(1);  % Lateral force at the front
    Fyr = u(2);  % Lateral force at the rear
    lf = u(3);   % Distance to front axle
    lr = u(4);   % Distance to rear axle
    Iz = u(5);   % Yaw moment of inertia

    % Small constant for numerical stability
    epsilon = 1e-6;

    % Ensure Fyf and Fyr are not zero but preserve their signs
    if abs(Fyf) < epsilon
        Fyf = sign(Fyf) * epsilon; 
    end
    if abs(Fyr) < epsilon
        Fyr = sign(Fyr) * epsilon; 
    end

    % Ensure Iz is not zero
    if abs(Iz) < epsilon
        Iz = epsilon; 
    end

    % Compute yaw_rate
    yaw_rate = (Fyf * lf - Fyr * lr) / Iz;

    % Ensure yaw_rate is not exactly zero but maintain its sign
    if abs(yaw_rate) < epsilon
        yaw_rate = sign(yaw_rate) * epsilon;
    end
end