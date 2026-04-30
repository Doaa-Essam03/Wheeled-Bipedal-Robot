% --- Physical Constants ---

m = 0.1  ;  %  wheel mass
M = 2.5  ;  %  Body Mass
r = 0.05 ;  %  wheel radius
L = 0.15 ;  %  length to CoM 
Jz = 0.03 ; %  body inertia 
Iw = 0.000045 ; %  wheel inertia
g = 9.81; % gravity

Ew = M + 2*m + (2*Iw/ (r^2))   ; 
Ep = (Jz + M*(L^2)) ; 
denometer = Ew * Ep - ( (M^2)*(L^2)) ; 

A21 = (Ew*M*g*L )/denometer ; 
B2 =  -1* (Ew + M*L/r) /denometer;
A41 = ( -1* (M^2)*g*(L^2) )/denometer ; 
B4 = ( (Ep/r) + M*L )/denometer ; 


A_Mat = [ 0     1    0     0 
          A21   0    0     0
          0     0    0     1 
          A41   0    0     0  ] ;

B_Mat = [ 0
          B2
          0 
          B4  ] ; 

C_Mat = [ 1  0    0   0 
          0  1    0   0
          0  0    1   0 
          0  0    0   1  ] ;

Q = diag([500, 1, 800, 150]);
R = 100  ; 

K = lqr(A_Mat, B_Mat, Q, R) 

