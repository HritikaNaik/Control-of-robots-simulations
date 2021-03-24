%% Run a Simulation of a Bang-bang controller to change the angle the robots is headed in from lecture 15
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
detector.maxRange =20;   % The radius of the disk is 12 units

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

pose = zeros(3,numel(tVec));  % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)   
    vref = 0.5; v =vref;
    
    % Update object detector and visualization
    detections = detector(pose(:,idx-1),objects);   % goal is detected
    viz(pose(:,idx-1),objects)  % Everything is plotted
    ylim([-5 15]);
    xlim([-5 15]);
    pause(0.25)
    
    if idx ==2 % To find the initial distance between robot and goal
        dist = detections(1);
    end
    
    position = pose(:,idx-1); % previous position
    v = vref*(detections(1))/dist;  %velocity
    
     %Since its a bang-bang controller, the change in the angle of the robot until it reaches the desired angle, is an arbitrary fixed value
    if  detections(2)> 0 % If the actual angle and desired angle have a positive difference, then we increase the angle by a positive amount
        theta = pi/6;
    elseif detections(2)< 0 % If the actual angle and desired angle have a negetive difference, then we increase the angle by a negetive amount
           theta = -pi/6;
        else
            theta = 0 % No need to change once the desired angle is reached
        end
    Kp = 0.4; % The constant of proportionality, based on the physical constraints of the system
    change = [v*cos(position(3)),v*sin(position(3)),Kp*theta]; % Change in position
    pose(:,idx)= change + [position(1) position(2) position(3)]; % Position updated
        
    if detections(1) < 0.05  % When its close enough to the goal, in terms of linear distance
        pose(:,idx)= [0 0 Kp*theta]+ [position(1) position(2) position(3)];
    end
    
    waitfor(r);
end