% =========================================================================
% TWSB Robot: Kinematic Fitting & Validation Script
% Added: Comparison plots for Leg Length and CG Offset accuracy
% =========================================================================
clear; clc; close all;

% --- 1. Physical Constants ---
l1 = 0.5; l2 = 0.15; l3 = 0.15; l4 = 0.1; l23 = 0.1; 
m1 = 0.2; m2 = 0.3; m3 = 0.1; m23 = 0.8;             
m_legs = m1 + m2 + m3 + m23;                         

% --- 2. Data Generation Loop ---
theta1_range = linspace(0.5, 1.5, 100); 
Lleg_data = zeros(size(theta1_range));
LW_data = zeros(size(theta1_range));

for i = 1:length(theta1_range)
    theta1 = theta1_range(i); 
    
    % Kinematic equations
    l5 = sqrt(l2^2 + l4^2 - 2 * l2 * l4 * cos(theta1 + pi/4));
    theta2 = asin(max(-1, min(1, (l4 * sin(theta1 + pi/4)) / l5)));
    theta3 = acos(max(-1, min(1, (l23^2 + l5^2 - l3^2) / (2 * l23 * l5))));
    theta4 = pi - theta1 - theta2 - theta3;
    theta5 = pi - theta4;
    theta6 = asin(max(-1, min(1, (l5 * sin(theta3)) / l3))) - theta4;
    
    lleg = l2 * sin(theta1) + l1 * sin(theta4);
    
    xc2 = (l2 * sin(theta1)) / 2;
    xc1 = l2 * sin(theta1) + (l1 * sin(theta5)) / 2;
    xc3 = -l4 * sin(pi/4) + (l3 * sin(theta6)) / 2;
    xc23 = l2 * sin(theta1) - (l23 * sin(theta4)) / 2;
    
    L0 = (m1*xc1 + m2*xc2 + m3*xc3 + m23*xc23) / m_legs;
    lw = lleg - L0; 
    
    Lleg_data(i) = lleg;
    LW_data(i) = lw;
end

% --- 3. Perform Quadratic Fittings ---
p_theta1_lleg = polyfit(theta1_range, Lleg_data, 2);
p_theta1_lw   = polyfit(theta1_range, LW_data, 2);

% Calculate fitted values for validation
Lleg_fit = polyval(p_theta1_lleg, theta1_range);
Lw_fit   = polyval(p_theta1_lw, theta1_range);

% --- 4. Original Visualization (Fitting Curves) ---
figure('Name', 'Fitting Analysis', 'Color', 'w', 'Position', [50, 400, 1200, 400]);
subplot(1, 3, 1);
plot(theta1_range, Lleg_data, 'ro', 'MarkerSize', 3); hold on;
plot(theta1_range, Lleg_fit, 'b-', 'LineWidth', 2);
grid on; title('\theta_1 \rightarrow l_{leg}'); ylabel('m');

subplot(1, 3, 2);
plot(Lleg_data, LW_data, 'go', 'MarkerSize', 3); hold on;
[lleg_sorted, idx] = sort(Lleg_data);
p_lleg_lw = polyfit(Lleg_data, LW_data, 2);
plot(lleg_sorted, polyval(p_lleg_lw, lleg_sorted), 'b-', 'LineWidth', 2);
grid on; title('l_{leg} \rightarrow l_w');

subplot(1, 3, 3);
plot(theta1_range, LW_data, 'mo', 'MarkerSize', 3); hold on;
plot(theta1_range, Lw_fit, 'b-', 'LineWidth', 2);
grid on; title('\theta_1 \rightarrow l_w');

% --- 5. Validation Window (Real vs Fit Comparison) ---
figure('Name', 'Accuracy Validation', 'Color', 'w', 'Position', [50, 50, 1000, 500]);

% Subplot A: Leg Length Comparison
subplot(2, 2, 1);
plot(theta1_range, Lleg_data, 'k', 'LineWidth', 2); hold on;
plot(theta1_range, Lleg_fit, 'r--', 'LineWidth', 1.5);
grid on; legend('Real (Eq)', 'Fitted');
ylabel('L_{leg} (m)'); title('Leg Length: Real vs Quadratic Fit');

% Subplot B: Leg Length Error
subplot(2, 2, 3);
plot(theta1_range, abs(Lleg_data - Lleg_fit), 'd-');
grid on; ylabel('Abs Error (m)'); xlabel('Servo Angle (rad)');
title('L_{leg} Fitting Residuals');

% Subplot C: Lw Comparison
subplot(2, 2, 2);
plot(theta1_range, LW_data, 'k', 'LineWidth', 2); hold on;
plot(theta1_range, Lw_fit, 'm--', 'LineWidth', 1.5);
grid on; legend('Real (Eq)', 'Fitted');
ylabel('L_w (m)'); title('CG Offset (Lw): Real vs Quadratic Fit');

% Subplot D: Lw Error
subplot(2, 2, 4);
plot(theta1_range, abs(LW_data - Lw_fit), 'd-', 'Color', [0.5 0 0.5]);
grid on; ylabel('Abs Error (m)'); xlabel('Servo Angle (rad)');
title('L_w Fitting Residuals');

% --- 6. Output Coefficients ---
fprintf('\n--- COPY THESE COEFFICIENTS TO YOUR ARDUINO CODE ---\n');
fprintf('// Fitting: theta1 -> Lleg\n');
fprintf('float a1 = %f; float b1 = %f; float c1 = %f;\n', p_theta1_lleg);
fprintf('\n// Fitting: Lleg -> Lw\n');
fprintf('float a2 = %f; float b2 = %f; float c2 = %f;\n', p_lleg_lw);
fprintf('\n// Fitting: theta1 -> Lw\n');
fprintf('float a3 = %f; float b3 = %f; float c3 = %f;\n', p_theta1_lw);