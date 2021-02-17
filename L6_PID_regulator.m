%% Run a Simulation of a PID regulator from lecture 6 that attempts to account for air resistance
% My aim is to design a cruise controler for a car to run at a given reference speed
% The first attempt lead to a very jerky result, although the desired velocity was reached
% The second attempt was thwarted by air resistance in reaching its goal
% The third attempt was not robust, as it was too dependant on arbitrary constants
% Here again, I assume that the accelaration (a) is directly proportional to the race or brake applied (u)
%% a = c * u / m - y * V         
% where m is the mass of the vehicle, c a fixed propery of the engine, y is the coefficient of air resistance and v is the velocity
% This time,  along with u being proportional to the difference between reference and actual speed (e), 
% we also make it proportional to its derivative at a point (e_dot) and its integral (E)
%% u = k_p * e + k_i * E + k_d * e_dot
clc
close all
clear all

global e_old;
global E;
e_old = 0;
E = 0;

tspan = [0 50];%Time interval of simulation
[t,vel] = ode45(@(t,v)Regulator(t,v),tspan,0);% obtaining the velocity at each time interval by integrating accelaration
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
    k_P = 6; % constant of proportionality
    k_I = 0.02;%constant of integral
    k_D = 1;% constatnt of derivative
    %% set K_D = 0 to get a PI regulator
    
    global e_old;
    global E;
    ref = 50; % reference velocity
    y = 0.15; %air resistance
    l = 0.05; % transmission coefficient
    
    e = ref - v; %error term
    E = E + e; %integral of error
    e_dot = e_old - e;%derivative of error
    u = k_P * e + k_D * e_dot + k_I * E;
    e_old = e;
    
    dv = l* u - y*v;  
    
end