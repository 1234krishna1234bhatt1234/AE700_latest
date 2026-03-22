%% gamma_sweep_test.m
aerosonde_parameters; 
Va_target = 200;
gamma_range = -5:1:5; 



fprintf('%-10s | %-12s | %-12s | %-12s | %-12s\n', ...
        'Gamma(deg)', 'Alpha(deg)', 'Theta(deg)', 'Throttle(%)', 'h_dot(m/s)');
fprintf('--------------------------------------------------------------------------------\n');

for g_deg = gamma_range
    g_rad = g_deg * pi/180;
    
    [xt, ut, dxt] = run_single_trim(g_rad, Va_target, MAV);
    
    alpha_deg = atan2(xt(6), xt(4)) * 180/pi;
    h_dot = -dxt(3); 
    
    fprintf('%-10d | %-12.2f | %-12.2f | %-12.2f | %-12.2f\n', ...
            g_deg, alpha_deg, xt(8)*180/pi, ut(4)*100, h_dot);
end


%% --- THE ISOLATED TRIM FUNCTION ---
function [x_trim, u_trim, dx_trim] = run_single_trim(gamma, Va, P)
    x0 = [0; 0; -500; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
    u0 = [0; 0; 0; 0.5];
    y0 = [Va; gamma; 0]; 
    iy = [1, 3]; 
    idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
    dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0];
    w = warning('off','all');
    opt = zeros(1, 18);
    opt(1)  = 0;     % Set to 1 if you want to see the solver's progress (helpful for debugging)
    opt(14) = 5000;  % Increase max function evaluations (Default is usually 200 or 500)
    
    % 3. Silencing warnings
    w = warning('off', 'all'); 
    
    % 4. Call trim with the 10th argument (opt)
    [x_trim, u_trim, ~, dx_trim] = trim('mavsim_trim', x0, u0, y0, [], [], iy, dx0, idx, opt);
    warning(w);
end