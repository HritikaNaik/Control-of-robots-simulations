%% Run a Simulation of a PID regulator to change the angle the robots is headed in from lecture 15
% My aim is to have the robot head to a goal
% The positions of the robot and goal are randomised so that it will have to change not only its x and y coordinates, but also the angle its pointed at to reach the goal
clear all
close all
clc

%% Create the environment of the simulation
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);   % Using an empty map of resolution 2

%% Create the goal
objects = [5*rand 5*rand 1]   % Position of object is random, its x and y coordinates can take any value from 0 to 5

%% Create object Detector sensor
detector = ObjectDetector
detector.fieldOfView = 2*pi; %As if there is a disk around it where eveything is known
detector.maxRange =12;   % The radius of the disk is 12 units

%% Create visualizer
viz = Visualizer2D;
attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1];   % Our goal will be red in colour
viz.objectMarkers = 'so^';

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
initPose = [4*rand; 4*rand; pi/3]        % Initial pose (x y theta)
% Initialize time, input, and pose array
tVec = 0:sampleTime:5;         % Time array
vxRef = zeros(size(tVec));      % Reference x speed
vyRef = zeros(size(tVec));  % Reference y speed
wRef = 0.75*ones(size(tVec));   % Reference angular speed
ref = [vxRef;vyRef;wRef];
pose = zeros(3,numel(tVec));  % Pose matrix
pose(:,1) = initPose;
Thet = 0; theta_old = 0;  %For the Integral and derivative terms of the PID regulator

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)   
    vref = 0.5; v = 0; Kp = 1.7; 
    % Put Kp = 7 to test the effect of low Kp
    Kd = 0.03; Ki = 0.01;
    % Kd and Ki can be set to 0 actually. I have neglected air resistance so they are not needed
    
    % Convert the reference speeds to world coordinates
    vel = bodyToWorld(ref(:,idx-1),pose(:,idx-1));
    
    % Update object detector and visualization
    detections = detector(pose(:,idx-1),objects)   % goal is detected
    viz(pose(:,idx-1),objects)  % Everything is plotted
    ylim([-3 10]);
    xlim([-3 10]);
    pause(0.25)
    
    if idx ==2 % To find the initial distance between robot and goal
        dist = detections(1);
    end
    
    position = pose(:,idx-1); % previous position
    v = vref*(detections(1))/dist;  %velocity
    
    %% PID for the angle
    theta = position(3) + detections(2);  % The error term
    theta_dot  = theta_old - theta;  % Derivative term
    Thet = Thet +theta;  % Integral term
    theta_old = theta;
    % I have used merely a PI regulator for the position as air resistance there is neglected
    
    change = [v*cos(position(3)),v*sin(position(3)),Kp*theta+Ki*Thet+Kd*theta_dot]; % Change in position
    pose(:,idx)= change + [position(1) position(2) 0]; % Position updated
        
    if detections(1) < 0.05  % When its close enough to the goal
        break
    end
    
    waitfor(r);
end