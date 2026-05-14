% =========================================================
% CRUISE CONTROL SYSTEM USING PID CONTROLLER
% Maintains constant vehicle speed under varying road slopes
% =========================================================

clc;
clear;
close all;

%% VEHICLE PARAMETERS
m = 1000;          % Mass of vehicle (kg)
b = 50;            % Drag coefficient
desired_speed = 20; % Desired speed (m/s)

%% SIMULATION PARAMETERS
dt = 0.01;         % Time step
T = 60;            % Total simulation time
t = 0:dt:T;

%% PID CONTROLLER GAINS
Kp = 800;
Ki = 40;
Kd = 100;

%% INITIAL CONDITIONS
v = zeros(size(t));      % Vehicle speed
u = zeros(size(t));      % Control force
e_prev = 0;
integral = 0;

%% ROAD DISTURBANCE (UPHILL / DOWNHILL)
disturbance = zeros(size(t));

for i = 1:length(t)
    
    % Uphill from 15s to 30s
    if t(i) >= 15 && t(i) < 30
        disturbance(i) = -200;
        
    % Downhill from 40s to 50s
    elseif t(i) >= 40 && t(i) < 50
        disturbance(i) = 150;
        
    else
        disturbance(i) = 0;
    end
end

%% MAIN SIMULATION LOOP
for i = 2:length(t)

    % Error calculation
    error = desired_speed - v(i-1);

    % Integral term
    integral = integral + error * dt;

    % Derivative term
    derivative = (error - e_prev) / dt;

    % PID Controller Output
    u(i) = Kp*error + Ki*integral + Kd*derivative;

    % Vehicle Dynamics
    dv = (u(i) - b*v(i-1) + disturbance(i)) / m;

    % Update speed
    v(i) = v(i-1) + dv*dt;

    % Store previous error
    e_prev = error;
end

%% PLOTS
figure;

subplot(2,1,1)
plot(t, v, 'LineWidth', 2)
hold on
yline(desired_speed, '--r', 'Desired Speed')
xlabel('Time (s)')
ylabel('Speed (m/s)')
title('Cruise Control System Performance')
grid on

subplot(2,1,2)
plot(t, disturbance, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Road Disturbance')
title('Road Condition Changes')
grid on

%% DISPLAY MESSAGE
disp('Simulation Completed Successfully')
