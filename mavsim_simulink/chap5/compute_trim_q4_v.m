%% trim_constant_turn.m
% Computes trim states for a constant altitude, coordinated turn.

% 1. Load Aircraft Parameters
aerosonde_parameters; 

% 2. Define Turn Specifications
n = 1.2;             % Load factor
C_L = 0.8;           % Lift coefficient (Chosen between 0.7 and 1.0)

% 3. Calculate Turn Physics
% Roll angle required for the load factor: n = 1/cos(phi)
phi_target = acos(1/n); 

% Airspeed required to generate the necessary lift: L = n*W = 0.5*rho*Va^2*S*C_L
Weight = MAV.mass * MAV.gravity;
Lift_required = n * Weight;
Va_target = sqrt((2 * Lift_required) / (MAV.rho * MAV.S_wing * C_L));

% Turn radius and Turn rate (psidot)
R = Va_target^2 / (MAV.gravity * tan(phi_target));
psidot = Va_target / R;

fprintf('--- Turn Specifications ---\n');
fprintf('Load Factor (n) : %.2f\n', n);
fprintf('Lift Coef (C_L) : %.2f\n', C_L);
fprintf('Req. Airspeed   : %.2f m/s\n', Va_target);
fprintf('Req. Roll Angle : %.2f deg\n', phi_target * 180/pi);
fprintf('Turn Radius     : %.2f m\n\n', R);

% 4. Setup Initial Guesses (12-State Euler)
% [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
% For a steady turn, pitch rate (q) and yaw rate (r) are non-zero
theta_guess = 5 * pi/180; % Slight pitch up to maintain altitude
p_guess = -psidot * sin(theta_guess);
q_guess = psidot * sin(phi_target) * cos(theta_guess);
r_guess = psidot * cos(phi_target) * cos(theta_guess);

x0 = [0; 0; -500; Va_target; 0; 0; phi_target; theta_guess; 0; p_guess; q_guess; r_guess];

% 5. Define Constraints
% We MUST force the solver to keep the roll angle at our calculated phi_target
ix = [7]; % Index 7 is phi

u0 = [0; 0; 0; 0.5];
iu = [];

% Assuming your model outputs y = [Va; alpha; beta]
y0 = [Va_target; 0; 0]; 
iy = [1, 3]; % Lock Va and force beta=0 (coordinated turn)

% 6. Define Derivative Targets
% Everything is zero EXCEPT heading rate (psidot)
dx0 = [0; 0; 0; 0; 0; 0; 0; 0; psidot; 0; 0; 0];

% Lock derivatives 3 through 12. 
% (1 and 2 are pn/pe which will constantly change in a turn)
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% 7. Run Trim Solver
% Disable warnings and messages for a clean output
opt = zeros(1, 18); opt(1) = -1; opt(14) = 5000;
w = warning('off', 'all');

try
    [x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim', x0, u0, y0, ix, iu, iy, dx0, idx, opt);
catch ME
    warning(w); rethrow(ME);
end
warning(w);

% 8. Validate and Output Results
residual = norm(dx_trim(idx) - dx0(idx));
fprintf('--- Trim Results ---\n');
fprintf('Residual        : %.2e\n', residual);
if residual > 1e-2
    fprintf('WARNING: Solver did not perfectly converge. Check physics bounds.\n');
end

fprintf('Trimmed Alpha   : %.2f deg\n', atan2(x_trim(6), x_trim(4)) * 180/pi);
fprintf('Trimmed Pitch   : %.2f deg\n', x_trim(8) * 180/pi);
fprintf('Trimmed Roll    : %.2f deg\n', x_trim(7) * 180/pi);
fprintf('Trimmed Inputs  : [Elev: %.2f deg] | [Ail: %.2f deg] | [Rud: %.2f deg] | [Thr: %.2f%%]\n', ...
    u_trim(1)*180/pi, u_trim(2)*180/pi, u_trim(3)*180/pi, u_trim(4)*100);

% 9. Update MAV struct for Simulink
MAV.pn0    = x_trim(1);
MAV.pe0    = x_trim(2);
MAV.pd0    = x_trim(3);
MAV.u0     = x_trim(4);
MAV.v0     = x_trim(5);
MAV.w0     = x_trim(6);
MAV.phi0   = x_trim(7);
MAV.theta0 = x_trim(8);
MAV.psi0   = x_trim(9);
MAV.p0     = x_trim(10);
MAV.q0     = x_trim(11);
MAV.r0     = x_trim(12);

% Assign the trim controls to the workspace so your Simulink inputs can use them
delta_e_trim = u_trim(1);
delta_a_trim = u_trim(2);
delta_r_trim = u_trim(3);
delta_t_trim = u_trim(4);
