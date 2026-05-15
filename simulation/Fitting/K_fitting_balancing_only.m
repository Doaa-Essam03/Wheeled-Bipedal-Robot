% =========================================================================
% Simplified LQR: Pitch Balancing ONLY (2-State System)
% =========================================================================
clear; clc;

% --- 1. Parameters ---
theta1_range = linspace(0.75, 1.25, 100); 
m_w = 0.2; mb = 0.9; ml = 0.4; mp = mb + 2*ml;
r = 0.130/2 ; g = 9.81; lb = 0.12; Jz_body = 0.033; Iw = 0.00025;

% Geometry Fittings
a_lw = -0.037861; b_lw = 0.088834; c_lw = 0.113857; 
a_lleg = -0.072848; b_lleg = 0.205408 ; c_lleg = 0.094574 ;

% LQR Weights for 2 states: [pitch, pitch_rate]
Q = diag([300, 10]); 
% R is now just a single value because we only have 1 control objective (Balance)
R = 300; 

K_data = [];

for t1 = theta1_range
    lw_curr = a_lw*t1^2 + b_lw*t1 + c_lw;
    L_curr = (lb*mb + lw_curr*(2*ml)) / mp; 
    Ip_curr = Jz_body + mb*(lb - L_curr)^2 + (2*ml)*(L_curr - lw_curr)^2; 
    
    % Dynamics Simplification for 2 States
    Ew = mp + 2*m_w + (2*Iw / (r^2)); 
    Ep = Ip_curr; 
    denom = Ew * Ep - (mp^2 * L_curr^2); 
    
    % A and B for [pitch; pitch_rate]
    A21 = (Ew * mp * g * L_curr) / denom; 
    B2  = -1 * (Ew + mp * L_curr / r) / denom;
    
    A_Reduced = [0   1;
                 A21 0];
             
    B_Reduced = [0;
                 B2];
    
    % Solve LQR
    K_curr = lqr(A_Reduced, B_Reduced, Q, R);
    K_data = [K_data; K_curr]; 
end

% --- 2. Fit and Print ---
p_k1 = polyfit(theta1_range, K_data(:,1), 2);
p_k2 = polyfit(theta1_range, K_data(:,2), 2);

fprintf('\n--- COPY TO ARDUINO (PITCH ONLY) ---\n');
fprintf('float k1_pitch = %f*t1^2 + %f*t1 + %f;\n', p_k1);
fprintf('float k2_rate  = %f*t1^2 + %f*t1 + %f;\n', p_k2);