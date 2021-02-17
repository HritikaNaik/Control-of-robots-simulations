%% Run a Simulation from lecture 22 demonstrating the rendevous problem with two robots
% Get two robots, assume the line joining them is the x axis, and get them to meet at a point.
clc
clear all;
close all;

%% Create environment
env = MultiRobotEnv(2);
load exampleMap
env.showTrajectory = true;

%% Create object Detector sensor
% One sensor is enough between two robots
detector = RobotDetector(env);
detector.sensorAngle = 0;
detector.fieldOfView = pi/2;
detector.maxRange = 12;
detector.maxDetections = 7;

%% Set random postions for both robots on the x axis, facing each other
robotPoses(:,1) = [6*rand 0 0];
robotPoses(:,2) = [6+6*rand 0 -pi]
env(1:2,robotPoses);
ylim([-1 2]);
xlim([0 12]);
pause(0.7);
detections = step(detector)

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
% Initialize time, input, and pose arrays
tVec = 0:sampleTime:1.5;         % Time array
vxRef = zeros(size(tVec));      % Reference x speed
vyRef = zeros(size(tVec));  % Reference y speed
wRef = 0.75*ones(size(tVec));   % Reference angular speed
ref = [vxRef;vyRef;wRef];

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 1:numel(tVec)  
    vel = 0.3; %  regular velocity
    
    % Update object detector and visualization
    env(1:2,robotPoses);
    ylim([-1 2]);
    xlim([0 12]);
    pause(0.75);
    detections = step(detector);
    
    % Change the position of the robots
    x = vel * detections(1);
    robotPoses(:,1) = robotPoses(:,1) + [x 0 0].';
    robotPoses(:,2) = robotPoses(:,2) + [-x 0 0].';
    
    if detections(1) <= 0.01   %  Once the robots meet, end loop
        break
    end
    
    waitfor(r);
end
env(1:2,robotPoses); %  Final positions
    