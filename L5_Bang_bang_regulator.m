%% Run a Simulation of a Bang Bang controler from lecture 5
% My aim is to design a cruise controler for a car to run at a given reference speed
% First attempt is by designing the most obvious type of conroller
% Here, we assume that the accelaration (a) is direction proportional to the race or brake applied (u)
%% a = c * u /m          
% where m is the mass of the vehicle and c a fixedpropery of the engine
% If umax is the maximum race or braking the car can manage
%% u = umax 
% if the car is slower than the reference velocity 
%% u = - umax
% if the car is faster than the reference velocity
%% u = 0 
% if it is at the reference velocity

close all
clc
clear all

tspan = 0:0.0005:100; %Time interval of simulation
[t,vel] = ode45(@(t,v)Controller(t,v),tspan,0); % obtaining the velocity at each time interval by integrating accelaration
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
function dv = Controller(t,v)
    l = 8; % arbitrary value of the ratio of transmission coefficient of the car to its mass, affects how fast speed increases
    ref = 50; % reference velocity 
    umax = 0.3; % maximum accelaration or deaccelaration
     e = ref - v; % error term
     
    if e == 0
        u = 0;
    elseif e > 0
       u = umax; 
    else
       u = -umax;
    end
                
    dv = l* u;  
end