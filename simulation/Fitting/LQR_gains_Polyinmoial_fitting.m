% =========================================================================
% LQR Gain Scheduling: theta1 -> K_matrix Fitting
% =========================================================================

% --- 1. System Constants & Range ---
% Define the sweep range for the servo angle (radians)
theta1_range = linspace(0.5, 1.5, 50); 

% Physical Parameters (Mass in kg, lengths in m)
m_w = 0.2;      % Mass of one wheel
mb = 0.9;       % Mass of the main body (chassis)
ml = 0.4;       % Mass of one leg assembly
mp = mb + 2*ml; % Total pendulum mass (Body + 2 legs)

r = 0.05;       % Wheel radius
g = 9.81;       % Gravity
D = 0.15;       % Track width (distance between wheels)
lb = 0.12;      % Body COM height relative to the hip joint
Jz_body = 0.033; % Constant pitch inertia of the chassis
Iw = 0.00025;   % Wheel rotational inertia

% Fitting Coefficients (Calculated from your geometry model)
% theta1 -> lw (Leg Center of Mass height)
a_lw = -0.010700; b_lw = 0.038542; c_lw = 0.345832; 
% theta1 -> lleg (Leg length from axle to hip)
a_lleg = -0.087522; b_lleg = 0.263838; c_lleg = 0.307096;

% LQR Weighting Matrices (Tune these for performance)
Q = diag([200, 10, 500, 10, 300, 10]); % [pitch, d_pitch, pos, vel, yaw, d_yaw]
R = diag([400, 300]);                 % [torque_balance, torque_steering]

% --- 2. Data Generation Loop ---
K_row1_data = []; % To store Balance/Forward gains
K_row2_data = []; % To store Steering/Yaw gains

for t1 = theta1_range
    % Step 1: Calculate current leg geometry via fitting
    lw_curr = a_lw*t1^2 + b_lw*t1 + c_lw;
    lleg_curr = a_lleg*t1^2 + b_lleg*t1 + c_lleg;
    
    % Step 2: Update Total System Center of Mass (L1)
    % L1 is the height from the axle to the COM of the whole system
    L_curr = (lb*mb + lw_curr*(2*ml)) / mp; 
    
    % Step 3: Update Pitch Inertia (Ip) using Parallel Axis Theorem
    % Ip is the resistance to falling forward/backward
    Ip_curr = Jz_body + mb*(lb - L_curr)^2 + (2*ml)*(L_curr - lw_curr)^2; 
    
    % Step 4: Update Yaw Inertia (Jy) dynamically
    % As the leg flexes, links move horizontally. 
    % Simplified: Jy = Jy_body + 2 * (m_leg * (Distance_from_Center)^2)
    % We assume a base Jy plus a variation based on leg extension
    Jy_base = 0.008; 
    Jy_curr = Jy_base + (2 * ml * (D/2)^2) * (lleg_curr / 0.45); % Example dynamic scaling
    
    % Step 5: Build State-Space Matrices (A and B)
    Ew = mp + 2*m_w + (2*Iw / (r^2)); 
    Ep = Ip_curr; 
    denom = Ew * Ep - (mp^2 * L_curr^2); 
    
    A21 = (Ew * mp * g * L_curr) / denom; 
    B2  = -1 * (Ew + mp * L_curr / r) / denom;
    A41 = (-1 * mp^2 * g * L_curr^2) / denom; 
    B4  = (Ep / r + mp * L_curr) / denom; 
    B6  = 1 / (r * (m_w * D + (Iw * D) / (r^2) + 2 * Jy_curr / D));
    
    % A_Mat: Describes how states change naturally
    A_Mat = [ 0     1    0     0   0   0
              A21   0    0     0   0   0
              0     0    0     1   0   0
              A41   0    0     0   0   0
              0     0    0     0   0   1 
              0     0    0     0   0   0 ];
    
    % B_Mat: Describes how control inputs (torque) affect the states
    B_Mat = [ 0    0
              B2   0
              0    0
              B4   0
              0    0
              0    B6 ];
    
    % Step 6: Solve LQR for this specific configuration
    K_curr = lqr(A_Mat, B_Mat, Q, R);
    K_row1_data = [K_row1_data; K_curr(1, :)]; % Balance gains
    K_row2_data = [K_row2_data; K_curr(2, :)]; % Steering gains
end

% --- 3. Generate Polynomial Fittings (theta1 -> K_gains) ---
p_k1 = polyfit(theta1_range, K_row1_data(:,1), 2); % Pitch
p_k2 = polyfit(theta1_range, K_row1_data(:,2), 2); % Pitch rate
p_k3 = polyfit(theta1_range, K_row1_data(:,3), 2); % Position
p_k4 = polyfit(theta1_range, K_row1_data(:,4), 2); % Velocity
p_k5 = polyfit(theta1_range, K_row2_data(:,5), 2); % Yaw
p_k6 = polyfit(theta1_range, K_row2_data(:,6), 2); % Yaw rate

% --- 4. Print Results ---
fprintf('\n--- Final Arduino Coefficients ---\n');
fprintf('k1 pitch = %f*t1^2 + %f*t1 + %f\n', p_k1);
fprintf('k2 Pitch Rate = %f*t1^2 + %f*t1 + %f\n', p_k2);
fprintf('k3 Position = %f*t1^2 + %f*t1 + %f\n', p_k3);
fprintf('k4 Position Rate = %f*t1^2 + %f*t1 + %f\n', p_k4);
fprintf('k5 yaw = %f*t1^2 + %f*t1 + %f\n', p_k5);
fprintf('k6 yaw Rate = %f*t1^2 + %f*t1 + %f\n', p_k6);