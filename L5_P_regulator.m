%% Run a Simulation of a PID regulator from lecture 5
% My aim is to design a cruise controler for a car to run at a given reference speed
% The first attempt lead to a very jerky result, although the desired velocity was reached
% Here again, I assume that the accelaration (a) is directly proportional to the race or brake applied (u)
%% a = c * u / m - y * V         
% where m is the mass of the vehicle, c a fixed propery of the engine, y is the coefficient of air resistance and v is the velocity
% This time, instead of a umax, we make u proportional to the difference between reference and actual speed (e)
%% u = k_p * e
% where k_p is the constant of proportionality.
% This will make it smoother
% While designing model dynamic, lets take into account the effect of air resistance, which is proportional to the velocity
% Last time, acceleration did not depend of the state of the vehicle, so this wasnt relevant
close all
clc
clear all

tspan = [0 50];%Time interval of simulation
[t,vel] = ode45(@(t,v)Regulator(t,v),tspan,0);% obtaining the velocity at each time interval by integrating accelaration
close all
figure

%% Plot the result
plot(t,vel)
ylim([0 80]);
title('Line Plot of the velocity verses time')
xlabel('Time (s)') 
ylabel('Velocity (m/s)')
grid on

%% Function that finds the accelaration at each instant of time and sends it to the ODE solver
function dv = Regulator(t,v)
    k = 20; %proportional term
    ref = 50;% reference velocity
    
    y = 0.15; %coefficient of air resistance
    l = 0.05;% arbitrary value of the ratio of transmission coefficient of the car to its mass, affects how fast speed increases
    
    e = ref - v;% error term
    u = k * e; %value of race or brake produced by the car
    
    dv = l*u - y*v;  
end