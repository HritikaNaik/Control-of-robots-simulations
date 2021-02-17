%% Run a Simulation from lecture 18 demonstrating 3 approaches to directing the robot
% Those approaches are pure avoidance, pure goal seeking and blended
clc
close all
clear all

%% Create environment
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,1);

%% Create Obstacle or goal or both as [x y label] where label  = 1 for goal and label = 2 for obstacle
objects = [3, 2.5, 1; 7, 5, 2];
       
%% Create object Detector sensor
detector = ObjectDetector
detector.fieldOfView = pi/4;
detector.maxRange = 2;

%% Create visualizer
viz = Visualizer2D;
attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1];
viz.objectMarkers = 'so^';

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
initPose = [4; 1; pi/3];        % Initial pose (x y theta)
% Initialize time, input, and pose arrays
tVec = 0:sampleTime:1.5;         % Time array
vxRef = zeros(size(tVec));      % Reference x speed
vyRef = zeros(size(tVec));  % Reference y speed
wRef = 0.75*ones(size(tVec));   % Reference angular speed
ref = [vxRef;vyRef;wRef];
pose = zeros(3,numel(tVec)); % Pose matrix
pose(:,1) = initPose;
dist = [0,0]; % it will store whether or not an object is detected and the original distance to it

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)   
    vref = 0.5; v = vref;  % velocities
    
    % Convert the reference speeds to world coordinates
    vel = bodyToWorld(ref(:,idx-1),pose(:,idx-1));
    
    % Update object detector and visualization
    detections = detector(pose(:,idx-1),objects);
    viz(pose(:,idx-1),objects)
    ylim([0 10]);
    xlim([0 10]);
    pause(0.25)
    
    position = pose(:,idx-1); %  Current position of robot
    
    if ~isempty(detections) & dist(1) == 0 % Detecting this object for the first time
        if detections(3) > 1
            %% Pure Avoidance
            pose(:,idx) =  [position(1),position(2),position(3) + detections(2) + pi]; 
        else
            %% Pure Goal Seeking
            pose(:,idx) =  [position(1),position(2), position(3) + detections(2) ];
            dist = [1,detections(1)];
        end
        %% For blended, uncomment this:
        %pose(:,idx) =  [a(1),a(2),position(3) + detections(2)+ pi/2];
    else 
        if dist(1) == 1 % If it are heading towards a goal
            v = vref*(detections(1))/dist(2); %  P regulator for velocity (not pID as air resistance is neglected) 
        end
        change = [v*cos(position(3)),v*sin(position(3)),0].'; % Change in velocity
        pose(:,idx)= change + position; %  Position updated
    end
    
    if dist(1) == 1 & detections(1) < 0.05  % When its close enough to the goal
        break
    end
    
    waitfor(r);
end