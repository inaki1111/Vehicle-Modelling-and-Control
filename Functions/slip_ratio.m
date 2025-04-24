function lambda = slip_ratio(u)
    % Inputs:
    % u(1) = R: Wheel radius
    % u(2) = omega: Wheel angular velocity
    % u(3) = vx: Longitudinal velocity
    % u(4) = mode: Mode (1 = braking, 2 = acceleration)

    R = u(1);        
    omega = u(2);    
    vx = u(3);        
    mode = u(4);      

    epsilon = 1e-6; % Small threshold for near-zero values

    if abs(vx) < epsilon
        vx = sign(vx) * epsilon;
    end
    if abs(omega) < epsilon
        omega = sign(omega) * epsilon;
    end

    if mode == 1  % 'braking'
        if abs(R * omega - vx) < epsilon || abs(vx) < epsilon
            lambda = 0;
        else
            lambda = (R * omega - vx) / vx;
        end
    elseif mode == 2  % 'acceleration'
        if abs(R * omega - vx) < epsilon || abs(R * omega) < epsilon
            lambda = 0;
        else
            lambda = (R * omega - vx) / (R * omega);
        end
    else
        lambda = 1;  
    end
end