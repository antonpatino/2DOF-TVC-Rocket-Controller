%% Rocket TVC LQR Control Design - Symbolic Linearization
% This script derives the A and B state-space matrices for a rocket
% using Thrust Vector Control (TVC).
% Based on Newton-Euler equations of motion.

%% ================== PART 1: SYMBOLIC LINEARIZATION ==================
% Symbolic Variables Definition
% States: [theta_pitch, theta_yaw, theta_roll, omega_pitch, omega_yaw, omega_roll]
syms tp ty tr op oy or 'real' 
% Inputs: [gamma_pitch, gamma_yaw] (TVC angles)
syms gp gy 'real'             
% Physical Parameters
syms Ixx Iyy Izz F_t r_tvc r_cp 'real' % I: Moment of Inertia, F_t: Thrust, r_tvc: CoG to TVC distance, r_cp: Cog to CoP
syms q_dyn A_p D_diam 'real'          % q: Dynamic pressure (1/2*rho*v^2), A_p: Frontal Area, D: Diameter
syms Cm_p Cm_y 'real'                 % Aerodynamic moment coefficients 
syms Cd Cl_p Cl_y 'real'              % Aerodynamic force coefficients (Drag, Pitch Lift and Yaw Lift) 

% Kinematic Equations (theta_dot = omega)
dtp = op; 
dty = oy;
dtr = or;

% Dynamic Equations (omega_dot)
% Thrust direction vector (U_gamma) based on TVC rotation 
u_gamma = [cos(gp)*cos(gy); 
          -sin(gp); 
           cos(gp)*sin(gy)];

% Thrust Torque (tau_T): r_tvc x F_thrust 
% Leverage arm r_vec in -X axis (from CoG to actuator) 
r_vec = [r_tvc; 0; 0]; 
F_vec = F_t * u_gamma;
tau_T = cross(r_vec, F_vec); 

% Aerodynamic Force Torque (tau_AF) 
% Force applied at CP: tau = r_cp x F_aerodynamic
r_cp_vec = [r_cp; 0; 0]; % CP position relative to CoG
F_a_body = q_dyn * A_p * [-Cd; Cl_p; Cl_y]; % Aerodynamic force in body frame 
tau_AF = cross(r_cp_vec, F_a_body);

% Aerodynamic Moment (tau_AM) 
% Represents the damping/restoring moments based on dynamic pressure and coefficients
tau_AM = q_dyn * A_p * D_diam * [0; Cm_p; Cm_y]; 

% Inertia Matrix and Gyroscopic Effect: omega x (I * omega)
I_mat = diag([Ixx, Iyy, Izz]);
omega_vec = [op; oy; or];
gyro_effect = cross(omega_vec, I_mat * omega_vec);

% Total Torque and Equation of Motion (Newton-Euler) 
% omega_dot = I^-1 * (Total_Torque - Gyroscopic_Effect)
Total_Torque = tau_T + tau_AM+tau_AF; 
omega_dot = inv(I_mat) * (Total_Torque - gyro_effect);

% State Vector and System Function f(x,u)
% Complete 6-state vector
x_6 = [tp; ty; tr; op; oy; or];
f = [dtp; dty; dtr; omega_dot(1); omega_dot(2); omega_dot(3)];

% Symbolic Jacobian Calculation (Linearization)
A_sym = jacobian(f, x_6);
B_sym = jacobian(f, [gp; gy]);

% Evaluate at Equilibrium Point (Linearize at 0)
% Substituting all states and inputs with 0 for small-angle approximation
A_6x6 = subs(A_sym, [tp, ty, tr, op, oy, or, gp, gy], [0, 0, 0, 0, 0, 0, 0, 0]);
B_6x2 = subs(B_sym, [tp, ty, tr, op, oy, or, gp, gy], [0, 0, 0, 0, 0, 0, 0, 0]);

% Extraction of 4x4 Reduced Matrix (Pitch & Yaw only)
% Indices for Pitch_ang(2), Yaw_ang(3), Pitch_vel(5), Yaw_vel(6)
idx = [2,3,5,6]; 
A_4x4 = A_6x6(idx, idx);
B_4x2 = B_6x2(idx, :);

% Calculate with the rocket physical values for numerical output
val_Ixx = 0.00008;   % kg*m^2
val_Iyy = 0.006941;   % kg*m^2
val_Izz = 0.006941;   % kg*m^2
val_Ft  = 12.0;   % Newtons (Thrust)
val_rtvc = 0.31;   % Meters (CoG to TVC)
val_rcp = 0.955;   % Meters (CoG to TVC)
val_q    = 0.5*1.204*28^2;  % Dynamic pressure
val_D    = 0.085;   % Diameter
val_Ap   = pi*val_D^2; % Area
val_Cmp  = -0.4;  % Damping coeff (Pitch)
val_Cmy  = -0.4;  % Damping coeff (Yaw)
Ts   = 0.02; %Sample time (s)
% Substitute values
A_num = double(subs(A_4x4, [Ixx, Iyy, Izz, F_t, r_tvc, q_dyn, A_p, D_diam, Cm_p, Cm_y], ...
    [val_Ixx, val_Iyy, val_Izz, val_Ft, val_rtvc, val_q, val_Ap, val_D, val_Cmp, val_Cmy]));
B_num = double(subs(B_4x2, [Ixx, Iyy, Izz, F_t, r_tvc, q_dyn, A_p, D_diam, Cm_p, Cm_y], ...
    [val_Ixx, val_Iyy, val_Izz, val_Ft, val_rtvc, val_q, val_Ap, val_D, val_Cmp, val_Cmy]));
C_num = [eye(2), zeros(2)]; % Output is Pitch and Yaw angles
D_num = zeros(2, 2);

%% ================== PART 2: LQR DESIGN ====================

% LQR Weights
Qx = diag([10, 10, 0.0001, 0.0001]); % Weights on [tp, ty, op, oy]
Qi = diag([300, 300]);       % Weights on integral error of [tp, ty]
R  = 1.5 * eye(2);           % Weight on control effort (gp, gy)

% Augment System for Integral Action (Discrete Domain)
sys_c = ss(A_num, B_num, C_num, D_num);
sys_d = c2d(sys_c, Ts, 'zoh');
A_d = sys_d.A; B_d = sys_d.B;

[n, m] = size(B_d); p = 2; % 2 integrators for 2 angles
A_aug = [A_d, zeros(n, p); -Ts * C_num, eye(p)];
B_aug = [B_d; zeros(p, m)];

% Compute DLQR Gain
Q_aug = blkdiag(Qx, Qi);
K_aug = dlqr(A_aug, B_aug, Q_aug, R);

% Split Gains
Kx = K_aug(:, 1:n);
Ki = -K_aug(:, n+1:end);

%% ================== PART 3: RESULTS OUTPUT ==========================
fprintf('--- Continuous-Time Matrices (Symbolic Derived) ---\n');
fprintf('--- Linearized 4x4 State Matrix (A) ---\n');
disp(A_num);
fprintf('--- Linearized 4x2 State Matrix (B) ---\n');
disp(B_num);
fprintf('--- Linearized 2x4 State Matrix (C) ---\n');
disp(C_num);
fprintf('--- Linearized 2x2 State Matrix (D) ---\n');
disp(D_num);
fprintf('--- Final Discrete LQR Gains (Ts = %.4f) ---\n', Ts);
fprintf('Kx (4x4):\n'); disp(Kx);
fprintf('Ki (2x2):\n'); disp(Ki);

% Verification: Check stability of closed-loop augmented system
cl_poles = eig(A_aug - B_aug * K_aug);
if all(abs(cl_poles) < 1)
    fprintf('Success: Discrete closed-loop system is STABLE.\n');
else
    warning('System is UNSTABLE. Check Q/R weights.');
end