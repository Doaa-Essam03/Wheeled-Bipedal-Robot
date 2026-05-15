% =========================================================================
% TWSB Robot: Dual Fitting Error Comparison
% 1. Direct Error: theta1 -> Fit -> Lw
% 2. Chained Error: theta1 -> Math(Lleg) -> Fit -> Lw
% =========================================================================
clear; clc; close all;

% --- 1. Physical Constants ---
l1 = 0.5; l2 = 0.15; l3 = 0.15; l4 = 0.1; l23 = 0.1; 
m1 = 0.2; m2 = 0.3; m3 = 0.1; m23 = 0.8;             
m_legs = m1 + m2 + m3 + m23;                         

% --- 2. Ground Truth Generation (Math Model) ---
theta1_range = linspace(0.5, 1.5, 200); 
Lleg_math = zeros(size(theta1_range));
Lw_math = zeros(size(theta1_range));

for i = 1:length(theta1_range)
    t1 = theta1_range(i); 
    
    % Kinematics
    l5 = sqrt(l2^2 + l4^2 - 2 * l2 * l4 * cos(t1 + pi/4));
    theta2 = asin(max(-1, min(1, (l4 * sin(t1 + pi/4)) / l5)));
    theta3 = acos(max(-1, min(1, (l23^2 + l5^2 - l3^2) / (2 * l23 * l5))));
    theta4 = pi - t1 - theta2 - theta3;
    theta5 = pi - theta4;
    theta6 = asin(max(-1, min(1, (l5 * sin(theta3)) / l3))) - theta4;
    
    lleg = l2 * sin(t1) + l1 * sin(theta4);
    xc2 = (l2 * sin(t1)) / 2;
    xc1 = l2 * sin(t1) + (l1 * sin(theta5)) / 2;
    xc3 = -l4 * sin(pi/4) + (l3 * sin(theta6)) / 2;
    xc23 = l2 * sin(t1) - (l23 * sin(theta4)) / 2;
    L0 = (m1*xc1 + m2*xc2 + m3*xc3 + m23*xc23) / m_legs;
    
    Lleg_math(i) = lleg;
    Lw_math(i) = lleg - L0; 
end

% --- 3. Polynomial Fitting ---
% Path 1: Direct Fitting (theta1 -> Lw)
p_theta_lw = polyfit(theta1_range, Lw_math, 2);

% Path 2: Chained Fitting (Lleg -> Lw)
p_lleg_lw = polyfit(Lleg_math, Lw_math, 2);

% --- 4. Error Calculation ---
% Prediction from Direct Path
Lw_direct_fit = polyval(p_theta_lw, theta1_range);
error_direct = abs(Lw_math - Lw_direct_fit);

% Prediction from Chained Path (Using math-calculated Lleg as input)
Lw_chained_fit = polyval(p_lleg_lw, Lleg_math);
error_chained = abs(Lw_math - Lw_chained_fit);

% --- 5. Visualization ---
figure('Name', 'Fitting Error Comparison', 'Color', 'w', 'Position', [100, 100, 1100, 600]);

% --- TOP: Direct Path Comparison ---
subplot(2, 2, 1);
plot(theta1_range, Lw_math, 'k', 'LineWidth', 2); hold on;
plot(theta1_range, Lw_direct_fit, 'r--', 'LineWidth', 1.5);
grid on; title('Direct Path: \theta_1 \rightarrow L_w');
ylabel('L_w (m)'); legend('Math Model', 'Direct Fit');

subplot(2, 2, 3);
area(theta1_range, error_direct, 'FaceColor', [1 0.8 0.8], 'EdgeColor', 'r');
grid on; title('Direct Fit Residuals (Error)');
ylabel('Abs Error (m)'); xlabel('\theta_1 (rad)');

% --- BOTTOM: Chained Path Comparison ---
subplot(2, 2, 2);
plot(theta1_range, Lw_math, 'k', 'LineWidth', 2); hold on;
plot(theta1_range, Lw_chained_fit, 'b--', 'LineWidth', 1.5);
grid on; title('Chained Path: \theta_1 \rightarrow L_{leg} \rightarrow L_w');
ylabel('L_w (m)'); legend('Math Model', 'Chained Fit');

subplot(2, 2, 4);
area(theta1_range, error_chained, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');
grid on; title('Chained Fit Residuals (Error)');
ylabel('Abs Error (m)'); xlabel('\theta_1 (rad)');

% --- 6. Results Summary ---
fprintf('--- ERROR PERFORMANCE SUMMARY ---\n');
fprintf('Direct Path (\x03B81->Lw)  | Max Error: %.6f m | Mean Error: %.6f m\n', max(error_direct), mean(error_direct));
fprintf('Chained Path (Lleg->Lw) | Max Error: %.6f m | Mean Error: %.6f m\n', max(error_chained), mean(error_chained));

fprintf('\n--- ARDUINO COEFFICIENTS ---\n');
fprintf('// DIRECT: Lw = a3*t1^2 + b3*t1 + c3\n');
fprintf('float a3 = %.8f; float b3 = %.8f; float c3 = %.8f;\n', p_theta_lw);
fprintf('\n// CHAINED: Lw = a2*Lleg^2 + b2*Lleg + c2\n');
fprintf('float a2 = %.8f; float b2 = %.8f; float c2 = %.8f;\n', p_lleg_lw);