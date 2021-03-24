%% Run a Simulation of the non-linear Unicycle model from lecture 23
% We want to demonstrate the non-linearity of the system
% We observe that the robot only reaches the goal from certain starting positions
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
tVec = 0:sampleTime:7;         % Time array

pose = zeros(3,numel(tVec));  % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
r = rateControl(1/sampleTime);
v = 4; K = 0.7;  %Constants which represent the system
syms t %t (time) is the variable of differentiation
reg = 'B'; % We can choose between P and Bang-bang. 
%% Enter P for P, B for Bang-bang
%% reg = input('Enter P for P, B for Bang-bang','s') 

for idx = 2:numel(tVec)   
    
    
    % Update object detector and visualization
    detections = detector(pose(:,idx-1),objects);   % goal is detected
    viz(pose(:,idx-1),objects)  % Everything is plotted
    ylim([-1 6]);
    xlim([-1 6]);
    pause(0.25)
    
    position = pose(:,idx-1); % previous position
    
    %% Bang bang or P controller for the angle
    theta = position(3);  % The error term
    if reg == 'P' % P controller
        w = K * detections(2);
    else % Bang bang controller
        if  detections(2) > 0
            w = pi/2;
        elseif detections(2) < 0
            w = -pi/2;
        else
            w = 0;
        end
    end

    
    Change = [v*cos(theta),v*sin(theta), w];
    change = int(Change,t,(idx-1)*sampleTime,idx*sampleTime); % the derivative terms are integrated
    % for more complicated non-linear functions, we would have to use the 'integer' function, which will numerically integrate using global adaptive quadrature and default error tolerances
    pose(:,idx)= change + position.'; % Position updated
        
    if detections(1) < 0.1  % When its close enough to the goal
        break
    end
    
    waitfor(r);
end