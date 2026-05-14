## Electric Vehicle Cruise Control System 🚗⚡

## Overview
This project focuses on designing and simulating an Electric Vehicle (EV) Cruise Control System using MATLAB and Simulink. The objective of the project is to maintain a constant vehicle speed under varying road conditions by implementing a PI/PID controller.

The system automatically adjusts the throttle input whenever the vehicle experiences disturbances such as uphill or downhill road conditions. The controller continuously monitors the speed error and provides a stable and smooth response.

---

## Problem Statement
Design a cruise control system to maintain constant vehicle speed under varying road conditions.

### Vehicle Dynamics

:contentReference[oaicite:0]{index=0}

- Input: Throttle Control  
- Output: Vehicle Speed  
- Disturbance: Slope introduced at t = 10s  

---

## Objectives
- Maintain constant vehicle speed
- Reduce steady-state error below 2%
- Keep overshoot below 5%
- Achieve smooth transient response
- Ensure stable operation under disturbances

---

## Features
- MATLAB-based simulation
- PI/PID controller implementation
- Vehicle speed monitoring
- Disturbance handling capability
- Stable and smooth response
- Graphical visualization of results

---

## Tools & Technologies Used
- MATLAB
- Simulink
- Control System Toolbox

---

## Hardware Requirements
- Laptop/PC
- Minimum 4 GB RAM
- Intel i3/i5 Processor or higher

---

## Software Requirements
- MATLAB
- Simulink
- Control System Toolbox
- Windows/Linux/macOS

---

## System Workflow
1. Model vehicle dynamics
2. Design PI/PID controller
3. Apply step input
4. Introduce disturbance
5. Analyze speed response
6. Evaluate controller performance

---

## PID Controller Equation

:contentReference[oaicite:1]{index=1}

Where:
- Kp = Proportional Gain
- Ki = Integral Gain
- Kd = Derivative Gain
- e(t) = Error signal

---

## Simulation Results
The simulation results show that the PID controller successfully maintains the desired vehicle speed even under varying road conditions. The controller minimizes overshoot, reduces steady-state error, and provides stable performance during disturbances.

---

## Applications
- Electric Vehicles
- Autonomous Vehicles
- Driver Assistance Systems
- Automotive Control Systems
- Intelligent Transportation Systems

## CODE:
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

---

## Future Enhancements
- Adaptive Cruise Control
- AI-based Speed Prediction
- Real-time Hardware Integration
- Sensor-based Automation
- Energy-efficient EV Systems

---

## Repository Structure

```bash
├── MATLAB_Code/
│   ├── cruise_control.m
│
├── Simulink_Model/
│   ├── cruise_control.slx
│
├── Results/
│   ├── output_graphs.png
│
└── README.md
```

---

## How to Run
1. Open MATLAB
2. Run the cruise_control.m file
3. Observe the speed response graphs
4. Analyze the controller performance

---

## Conclusion
This project demonstrates a reliable Electric Vehicle Cruise Control System capable of maintaining constant speed under varying road conditions. The PID controller provides accurate and stable speed regulation, making the system suitable for modern intelligent vehicle applications.

---

## Author
Spoorthi  
Electronics & Communication Engineering
