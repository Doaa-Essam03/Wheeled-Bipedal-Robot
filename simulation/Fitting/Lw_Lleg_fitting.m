% =========================================================================
% TWSB Robot: Kinematic Fitting & CG Analysis Script
% This script calculates the relationship between Servo Angle, Leg Length, 
% and Center of Gravity (CG) Offset LW for a four-link leg mechanism.
% =========================================================================

clear; clc; close all;

% --- 1. Physical Constants (Adjust these to your actual robot) ---
l1 = 0.138 ; l2 = 0.106 ; l3 = 0.077; l4 = 0.075; l23 = 0.050; % Link lengths
m1 = 0.2; m2 = 0.3; m3 = 0.1; m23 = 0.8;             % Link masses
m_legs = m1 + m2 + m3 + m23;                         % Total leg mass

% --- 2. Data Generation Loop ---
theta1_range = linspace(0.75, 1.25 , 100); % Servo angle range (radians) 30:150
Lleg_data = zeros(size(theta1_range));
LW_data = zeros(size(theta1_range));

for i = 1:length(theta1_range)
    theta1 = theta1_range(i); 
    
    % Kinematic equations for the four-link mechanism
    l5 = sqrt(l2^2 + l4^2 - 2 * l2 * l4 * cos(theta1 + pi/4));
    theta2 = asin(max(-1, min(1, (l4 * sin(theta1 + pi/4)) / l5)));
    theta3 = acos(max(-1, min(1, (l23^2 + l5^2 - l3^2) / (2 * l23 * l5))));
    theta4 = pi - theta1 - theta2 - theta3;
    theta5 = pi - theta4;
    theta6 = asin(max(-1, min(1, (l5 * sin(theta3)) / l3))) - theta4;
    
    % Vertical projection (Leg Length)
    lleg = l2 * sin(theta1) + l1 * sin(theta4);
    
    % Vertical coordinates of center of mass for each link
    xc2 = (l2 * sin(theta1)) / 2;
    xc1 = l2 * sin(theta1) + (l1 * sin(theta5)) / 2;
    xc3 = -l4 * sin(pi/4) + (l3 * sin(theta6)) / 2;
    xc23 = l2 * sin(theta1) - (l23 * sin(theta4)) / 2;
    
    % Total leg center of gravity height (L0) and offset (lw)
    L0 = (m1*xc1 + m2*xc2 + m3*xc3 + m23*xc23) / m_legs;
    lw = lleg - L0; 
    
    Lleg_data(i) = lleg;
    LW_data(i) = lw;
end

% --- 3. Perform Quadratic Fittings (y = ax^2 + bx + c) ---

% Fitting 1: theta1 -> lleg
p_theta1_lleg = polyfit(theta1_range, Lleg_data, 2);

% Fitting 2: theta1 -> lw
p_theta1_lw = polyfit(theta1_range, LW_data, 2);

% --- 4. Visualization ---
figure('Color', 'w', 'Position', [100, 100, 1200, 400]);

% Subplot 1: theta1 vs lleg
subplot(1, 2, 1);
plot(theta1_range, Lleg_data, 'ro', 'MarkerSize', 3); hold on;
plot(theta1_range, polyval(p_theta1_lleg, theta1_range), 'b-', 'LineWidth', 2);
grid on; xlabel('Servo Angle \theta_1 (rad)'); ylabel('Leg Length l_{leg} (m)');
title('\theta_1 \rightarrow l_{leg}');
legend('Data', 'Fit', 'Location', 'best');

% Subplot 2: lleg vs lw
subplot(1, 2, 2);
plot(theta1_range, LW_data, 'mo', 'MarkerSize', 3); hold on;
plot(theta1_range, polyval(p_theta1_lw, theta1_range), 'b-', 'LineWidth', 2);
grid on; xlabel('Servo Angle \theta_1 (rad)'); ylabel('CG Offset l_w (m)');
title('\theta_1 \rightarrow l_w');
legend('Data', 'Fit', 'Location', 'best');

% --- 5. Output Coefficients for Arduino ---
fprintf('\n--- COPY THESE COEFFICIENTS TO YOUR ARDUINO CODE ---\n');
fprintf('// Fitting: theta1 -> Lleg\n');
fprintf('float a1 = %f; float b1 = %f; float c1 = %f;\n', p_theta1_lleg);

fprintf('\n// Fitting: theta1 -> Lw\n');
fprintf('float a3 = %f; float b3 = %f; float c3 = %f;\n', p_theta1_lw);