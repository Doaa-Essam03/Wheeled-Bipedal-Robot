% --- Physical Constants ---

%  XY motion with balancing 
%  X state vector  = [ Θ  Θ_dot  x  x_dot  δ  δ_dot ] 
%  J cost function = 0∫∞ (xTQx+uTRu) dt


m = 0.9  ;  %  wheel mass
M = 2.5  ;  %  Body Mass
r = 0.05 ;  %  wheel radius
L = 0.15 ;  %  length to CoM 
Jz = 0.03 ; %  body inertia about the yaw axis (horizontal)
Jy = 0.005; % Moment of inertia about the yaw axis (vertical)
Iw = 0.000045 ; %  wheel inertia
g = 9.81;   % gravity
D = 0.15;   % track width 

Ew = M + 2*m + (2*Iw/ (r^2))   ; 
Ep = (Jz + M*(L^2)) ; 
denometer = Ew * Ep - ( (M^2)*(L^2)) ; 

A21 = (Ew*M*g*L )/denometer ; 
B2 =  -1* (Ew + M*L/r) /denometer;
A41 = ( -1* (M^2)*g*(L^2) )/denometer ; 
B4 = ( (Ep/r) + M*L )/denometer ; 
B6 =  1 /(r*(m*D + (Iw*D)/(r^2) + 2*Jy/D)) ;

A_Mat = [ 0     1    0     0   0   0
          A21   0    0     0   0   0
          0     0    0     1   0   0
          A41   0    0     0   0   0
          0     0    0     0   0   1 
          0     0    0     0   0   0 ] ;

B_Mat = [ 0    0
          B2   0
          0    0
          B4   0
          0    0
          0    B6] ; 

C_Mat = [ 1  0    0   0   0  0
          0  1    0   0   0  0
          0  0    1   0   0  0
          0  0    0   1   0  0
          0  0    0   0   1  0
          0  0    0   0   0  1  ] ;

D_Mat = [ 0    0
          0    0
          0    0
          0    0
          0    0
          0    0] ;

Q = diag([200, 0, 500, 0 , 300 , 0 ]);
R = diag([400  , 300 ]) ; 

K = lqr(A_Mat, B_Mat, Q, R) ;

