% =========================================================================
% LQR Validation Script: Exact Gains vs. Polynomial Fitting
% =========================================================================

clear; clc; close all;

% --- 1. System Constants ---
theta1_range = linspace(0.5, 2.5, 50); 
m_w = 0.2; mb = 0.9; ml = 0.4; mp = mb + 2*ml;
r = 0.05; g = 9.81; D = 0.15; lb = 0.12;
Jz_body = 0.033; Iw = 0.00025;
Jy_base = 0.008;

% Geometry fittings (Inputs)
a_lw = -0.010700; b_lw = 0.038542; c_lw = 0.345832; 
a_lleg = -0.087522; b_lleg = 0.263838; c_lleg = 0.307096;

% LQR Weights
Q = diag([200, 10, 500, 10, 300, 10]);
R = diag([400, 300]);

% --- 2. Step-by-Step Mathematical Calculation (Exact Data) ---
K_exact = zeros(length(theta1_range), 6); % To store 6 gains for each angle

for i = 1:length(theta1_range)
    t1 = theta1_range(i);
    
    % Update physics exactly for this step
    lw = a_lw*t1^2 + b_lw*t1 + c_lw;
    lleg = a_lleg*t1^2 + b_lleg*t1 + c_lleg;
    L = (lb*mb + lw*(2*ml)) / mp; 
    Ip = Jz_body + mb*(lb - L)^2 + (2*ml)*(L - lw)^2; 
    Jy = Jy_base + (2 * ml * (D/2)^2) * (lleg / 0.45); 

    % Matrix construction
    Ew = mp + 2*m_w + (2*Iw / (r^2)); 
    Ep = Ip; 
    denom = Ew * Ep - (mp^2 * L^2); 
    
    A21 = (Ew * mp * g * L) / denom; 
    B2  = -1 * (Ew + mp * L / r) / denom;
    A41 = (-1 * mp^2 * g * L^2) / denom; 
    B4  = (Ep / r + mp * L) / denom; 
    B6  = 1 / (r * (m_w * D + (Iw * D) / (r^2) + 2 * Jy / D));
    
    A = [0 1 0 0 0 0; A21 0 0 0 0 0; 0 0 0 1 0 0; A41 0 0 0 0 0; 0 0 0 0 0 1; 0 0 0 0 0 0];
    B = [0 0; B2 0; 0 0; B4 0; 0 0; 0 B6];
    
    % The Mathematical Truth
    K_temp = lqr(A, B, Q, R);
    K_exact(i, :) = [K_temp(1,1:4), K_temp(2,5:6)]; % Store relevant gains
end

% --- 3. Create Polynomial Fittings ---
titles = {'k1 (Pitch)', 'k2 (Pitch Rate)', 'k3 (Position)', 'k4 (Velocity)', 'k5 (Yaw)', 'k6 (Yaw Rate)'};
p_coeffs = cell(1,6);

figure('Color', 'w', 'Name', 'LQR Fitting Validation');

for j = 1:6
    % Fit the polynomial to the exact data
    p_coeffs{j} = polyfit(theta1_range, K_exact(:,j), 2);
    
    % Generate fitted line for plotting
    K_fit = polyval(p_coeffs{j}, theta1_range);
    
    % Plotting
    subplot(2, 3, j);
    plot(theta1_range, K_exact(:,j), 'ro', 'MarkerSize', 4, 'DisplayName', 'Math (Exact)'); hold on;
    plot(theta1_range, K_fit, 'b-', 'LineWidth', 2, 'DisplayName', 'Fitting (Arduino)');
    grid on; title(titles{j});
    xlabel('\theta_1 (rad)'); ylabel('Gain Value');
    if j == 1, legend('Location', 'best'); end
end