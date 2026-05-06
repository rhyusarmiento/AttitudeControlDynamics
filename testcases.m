%{
Geometry Type	Dominant State	Physical Driver
Symmetric / Low Offset	Stable Libration	Restoring Torque (The Spring)
Intermediate Axis Heavy	Chaotic Tumbling	Inertia Cross-Coupling (The Saddle)
Asymmetric (Anemometer)	Constant Acceleration	Net Work Extraction (The Motor)
High Spin / Max Axis	Precessional Beats	Gyroscopic Stiffness (The Gyro)
%}

%% Beating of 2 axis

% state variables
M = 5;
q_raw = [1; 0; 0; 0]; 
q0 = q_raw / norm(q_raw);
w0 = [0.001; 0.001; 0.001]; 
X0 = [q0; w0];
a = 100; b = 100; c = 100;
offx = -20; offy = 0; offz = 0;
paddleoff = 0; m_p = 0; 
geom = [ 1, 0, 0,  b*c,  a/2 + offx, 0 + offy, 0 + offz;   % +X
         -1, 0, 0,  b*c, -a/2 + offx, 0 + offy, 0 + offz;   % -X
         0, 1, 0,  a*c,  0 + offx, b/2 + offy, 0 + offz;       % +Y
         0, -1, 0,  a*c,  0 + offx, -b/2 + offy, 0 + offz;      % -Y
         0, 0, 1,  a*b,  0 + offx, 0 + offy, c/2 + offz;       % +Z
         0, 0, -1,  a*b,  0 + offx, 0 + offy, -c/2 + offz];     % -Z

%% stable weather vane 

% state variables
M = 5;
q_raw = [1; 0; 0; 0]; 
q0 = q_raw / norm(q_raw);
w0 = [0.001; 0.001; 0.001]; 
X0 = [q0; w0];
a = 10; b = 4; c = 2;
offx = -3; offy = 0; offz = 0;
paddleoff = 0; m_p = 0; 
geom = [ 1, 0, 0,  b*c,  a/2 + offx, 0 + offy, 0 + offz;   % +X
         -1, 0, 0,  b*c, -a/2 + offx, 0 + offy, 0 + offz;   % -X
         0, 1, 0,  a*c,  0 + offx, b/2 + offy, 0 + offz;       % +Y
         0, -1, 0,  a*c,  0 + offx, -b/2 + offy, 0 + offz;      % -Y
         0, 0, 1,  a*b,  0 + offx, 0 + offy, c/2 + offz;       % +Z
         0, 0, -1,  a*b,  0 + offx, 0 + offy, -c/2 + offz];     % -Z

%% 5 accelerate

% state variables
M = 5;
q_raw = [1; 0; 0; 0]; 
q0 = q_raw / norm(q_raw);
w0 = [0.00; 0.00; 0.001]; 
X0 = [q0; w0];
a = 0.1; b = 4; c = 4;
offx = 0; offy = 0; offz = 0;
paddleoff = 25; m_p = 0.1; 
geom = [ 1, 0, 0,  b*c,  a/2 + offx, 0 + offy, 0 + offz;   % +X
         -1, 0, 0,  b*c,  -a/2 + offx, 0 + offy, 0 + offz;   % -X
         0, 1, 0,  a*c,  0 + offx, b/2 + offy, 0 + offz;       % +Y
         0, -1, 0,  a*c,  0 + offx, -b/2 + offy, 0 + offz;      % -Y
         1, 0, 0,  40,  0 + offx, paddleoff + offy, 0 + offz;       % +Z
         -1, 0, 0,  5,  0 + offx, paddleoff + offy, 0 + offz];     % -Z


%% chaos

% state variables
M = 5;
q_raw = [1; 0; 0; 0]; 
q0 = q_raw / norm(q_raw);
w0 = [0.001; 0.001; 0.001]; 
X0 = [q0; w0];
a = 10; b = 2; c = 5;
offx = 2.5; offy = 0; offz = 0;
paddleoff = 0; m_p = 0; 
geom = [ 1, 0, 0,  b*c,  a/2 + offx, 0 + offy, 0 + offz;   % +X
         -1, 0, 0,  b*c,  -a/2 + offx, 0 + offy, 0 + offz;   % -X
         0, 1, 0,  a*c,  0 + offx, b/2 + offy, 0 + offz;       % +Y
         0, -1, 0,  a*c,  0 + offx, -b/2 + offy, 0 + offz;      % -Y
         0, 0, 1,  b*a,  0 + offx, 0 + offy, c/2 + offz;       % +Z
         0, 0, -1,  b*a,  0 + offx, 0 + offy, -c/2 + offz];      % -Z

