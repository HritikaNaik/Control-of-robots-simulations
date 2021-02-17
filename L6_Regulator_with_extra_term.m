%% Run a Simulation of a PID regulator from lecture 6 that attempts to account for air resistance
% My aim is to design a cruise controler for a car to run at a given reference speed
% The first attempt lead to a very jerky result, although the desired velocity was reached
% The second attempt was thwarted by air resistance in reaching its goal
% Here again, I assume that the accelaration (a) is directly proportional to the race or brake applied (u)
%% a = c * u / m - y * V         
% where m is the mass of the vehicle, c a fixed propery of the engine, y is the coefficient of air resistance and v is the velocity
% This time, along with u being proportional to the difference between reference and actual speed (e), we add another term
%% u = k_p * e + y * m* v / c
% where the extra term makes an allowance for air resistance
clc
close all
clear all

tspan = [0 50];%Time interval of simulation
[t,vel] = ode45(@(t,v)Regulator(t,v),tspan,0);
close all

%% Plot the result
figure
plot(t,vel)
ylim([0 80]);
title('Line Plot of the velocity verses time')
xlabel('Time (s)') 
ylabel('Velocity (m/s)')
grid on

%% Function that finds the accelaration at each instant of time and sends it to the ODE solver
function dv = Regulator(t,v)
    k = 6;
    ref = 50;
    y = 0.15;
    l = 0.05;
    
    e = ref - v;
    u = k * e + y/l *v;
    
    dv = l* u - y*v;  
end