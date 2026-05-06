clc; clear; close all;

% parameters 
% environ constant
rho = 1.225e-12;
v_orb = 7800;
Cd = 2.2;

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

% Inertia
I_bus = diag([ (1/12)* M *(b^2+c^2), (1/12)* M *(a^2+c^2), (1/12)* M *(a^2+b^2) ]);
r_p = [0; paddleoff; 0];
S = [0 -r_p(3) r_p(2); r_p(3) 0 -r_p(1); -r_p(2) r_p(1) 0];
I_paddle = -m_p * (S * S); 
I = I_bus + I_paddle;

% time
tspan = [0 10000];

% engine
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, X] = ode45(@(t, X) spacecraft_dynamics(t, X, I, rho, v_orb, Cd, geom), tspan, X0, options);

% normalize state
q_out = X(:, 1:4);
q_squared = q_out.^2;
sum_sq = sum(q_squared, 2);
mag = sqrt(sum_sq);
q_out = q_out ./ mag;

% plotting
figure('Name', 'Spacecraft Attitude Simulation');

% Subplot 1: Quaternions (Orientation)
subplot(2,1,1);
plot(t, q_out, 'LineWidth', 1.5);
grid on;
title('Attitude Orientation (Quaternions)');
ylabel('Unitless Value');
legend('q_w', 'q_x', 'q_y', 'q_z');
ylim([-1.1 1.1]); % Quaternions stay between -1 and 1

% Subplot 2: Angular Velocity (Rotational Speed)
subplot(2,1,2);
plot(t, X(:, 5:7), 'LineWidth', 1.5);
grid on;
title('Angular Velocity (Body Frame)');
xlabel('Time (seconds)');
ylabel('Velocity (rad/s)');
legend('\omega_x', '\omega_y', '\omega_z');

% Total Rotational Kinetic Energy
% Used for the "Windmill" case (to show a parabola) and "Beating" (to show flatline)
% Calculate Kinetic Energy: T = 0.5 * w' * I * w at each time step
T_energy = zeros(length(t), 1);
for i = 1:length(t)
    w_vec = [X(i,5); X(i,6); X(i,7)];
    T_energy(i) = 0.5 * w_vec' * I * w_vec;
end
figure('Name', 'Rotational Kinetic Energy', 'Color', 'w');
plot(t, T_energy, 'k', 'LineWidth', 2);
grid on;
title('System Kinetic Energy over Time');
xlabel('Time (s)');
ylabel('Rotational Kinetic Energy (Joules)');
% Windmill: Should look like an accelerating curve (x^2).
% Beating: Should be a perfectly flat horizontal line (energy conserved).

% 3D Phase Space
% Used to contrast the "Weather Vane" (closed loop) with "Chaos" (Strange Attractor)
figure('Name', '3D Phase Space Topology', 'Color', 'w');
plot3(X(:,5), X(:,6), X(:,7), 'm', 'LineWidth', 1.2);
grid on; hold on;
% Mark the starting state
plot3(X(1,5), X(1,6), X(1,7), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8); 
title('3D Phase Space: \omega_x vs \omega_y vs \omega_z');
xlabel('\omega_x (Roll) [rad/s]');
ylabel('\omega_y (Pitch) [rad/s]');
zlabel('\omega_z (Yaw) [rad/s]');
legend('Phase Trajectory', 'Initial State', 'Location', 'Best');
view(3);
