%% Run a Simulation from lecture 26 demonstrating the rendevous problem with a swarm of robots
% Get many robots, and get them to meet at a point. We can choose the number of robots
clc
clear all;
close all;

%% Create environment
numRobots = 6;
env = MultiRobotEnv(numRobots);
env.showTrajectory = true; % So  we can see the path taken by the robots
load exampleMap

%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots % The robots are initialised to have a sensor each, which can see upto 8 units, all around them
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 8;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false;

%% Set random postions for both robots on the x axis, facing each other
robotPoses = 5*(rand(3,numRobots).*[1;1;pi]); % the initial poses of the robots are randomised
x_ = mean(robotPoses(1,:)) % x coordinate of the centroid
y_ = mean(robotPoses(2,:)) % y  coordinate of the centroid

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
% Initialize time: Simulation will last for 1.5 units of time, checking every 0.05 units 
tVec = 0:sampleTime:1.5;         % Time array

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 1:numel(tVec)  
    vref = 0.3; Kp = 0.3; % regulator values for linear and angular velocity, based on assumed physical contraints like friction
    
    % Update object detector and visualization
    env(1:numRobots,robotPoses); %shows picture
    xlim([-1 6]);   
    ylim([-1 6]);
    pause(0.75);
    
    for rIdx = 1:numRobots %running for each robot
        position = robotPoses(:,rIdx); %previous position ( x_coord, y_coord, angle_wrt_origin)
        x = position(1); % x coordinate of the robot
        y = position(2); % x coordinate of the robot
        angle = position(3);
        if abs(angle) < 0.1
            angle = 0.5;
        end
        theta_ = atan2(x_-x,y_-y); %now to get angle between 0 and pi, since we will be adding it to the robot's angle later
        if theta_<0 
            theta = pi + theta_;
        else
            theta = theta_;
        end
        
        change = [vref*(x_-x)*cos(angle) vref*(y_-y)*sin(angle) Kp*(theta+angle)]; % Using a P regulator
        robotPoses(:,rIdx) = [x y 0].' + change.'; %update position
     
    end
    
    waitfor(r);
end
env(1:numRobots,robotPoses);