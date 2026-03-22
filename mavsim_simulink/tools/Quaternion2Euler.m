function [phi, theta, psi] = Quaternion2Euler(quat)
    % Unpack for clarity
    e0 = quat(1);
    e1 = quat(2);
    e2 = quat(3);
    e3 = quat(4);

    % --- 1. Pitch (Theta) Protection ---
    % The argument for asin must be strictly between -1 and 1.
    % High-speed trim solvers often "over-step" and create 1.00001,
    % which produces complex numbers and crashes the solver.
    
    sin_theta = 2 * (e0 * e2 - e3 * e1);
    
    % Hard clip to [-1, 1]
    if sin_theta > 1.0
        sin_theta = 1.0;
    elseif sin_theta < -1.0
        sin_theta = -1.0;
    end
    
    theta = asin(sin_theta);

    % --- 2. Singularity Check (Gimbal Lock) ---
    % If theta is +/- 90 degrees, phi and psi become undefined (divide by zero).
    % We add a tiny epsilon (1e-6) to the denominators of atan2.
    
    eps_val = 1e-6;

    % Roll (Phi)
    num_phi = 2 * (e0 * e1 + e2 * e3);
    den_phi = 1 - 2 * (e1^2 + e2^2);
    % We use atan2(y, x). atan2 handles den=0 better than atan, 
    % but we ensure both aren't zero simultaneously.
    phi = atan2(num_phi, den_phi + eps_val);

    % Yaw (Psi)
    num_psi = 2 * (e0 * e3 + e1 * e2);
    den_psi = 1 - 2 * (e2^2 + e3^2);
    psi = atan2(num_psi, den_psi + eps_val);
end