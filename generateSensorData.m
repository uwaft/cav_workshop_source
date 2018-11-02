function [allData, scenario, sensors] = generateSensorData()
%generateSensorData - Returns sensor detections
%    allData = generateSensorData returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = generateSensorData optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.5 and Automated Driving System Toolbox 1.3.
% Generated on: 01-Nov-2018 20:17:03

% Create the drivingScenario object and ego car
[scenario, egoCar] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {});
running = true;
while running
    
    % Generate the target poses of all actors relative to the ego car
    poses = targetPoses(egoCar);
    time  = scenario.SimulationTime;
    
    objectDetections = {};
    laneDetections   = [];
    isValidTime      = false(1, numSensors);
    
    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        [objectDets, numObjects, isValidTime(sensorIndex)] = sensors{sensorIndex}(poses, time);
        objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
    end
    
    % Aggregate all detections into a structure for later use
    if any(isValidTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections',   {laneDetections}); %#ok<AGROW>
    end
    
    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = radarDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 50, ...
    'DetectionProbability', 0.8, ...
    'ActorProfiles', profiles);
sensors{2} = radarDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0 0.9], ...
    'Yaw', 90, ...
    'MaxRange', 50, ...
    'FieldOfView', [90 5], ...
    'ActorProfiles', profiles);
numSensors = 2;

function [scenario, egoCar] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [1.4 21.4 0;
    8.6 21.8 0;
    17.2 21.8 0;
    19.9 16.2 0;
    15.2 13.1 0;
    8.5 11.3 0;
    4.2 10.4 0;
    2.4 6.1 0;
    3.7 2.9 0;
    12 1.4 0;
    17.8 3.8 0;
    22.4 10.2 0;
    29.1 14.3 0;
    33.8 12 0;
    35.8 5.2 0;
    36.2 -0.9 0;
    33.3 -2.7 0;
    27.2 -3.2 0;
    21.1 -3.5 0;
    18.1 -7.6 0;
    21.6 -14.1 0;
    30.5 -18 0];
road(scenario, roadCenters);

% Add the ego car
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [3.2 20.1 0]);
waypoints = [3.2 20.1 0;
    6.8 20.2 0;
    9.6 20.4 0;
    13.3 21 0;
    16.3 20.6 0;
    17.5 18.9 0;
    17.7 16.6 0;
    16.2 15.5 0;
    12.5 14.6 0;
    8.6 13.1 0;
    5.3 12.5 0;
    2.7 10.9 0;
    1.4 6.8 0;
    2 2.8 0;
    3.5 0.6 0;
    7.6 -0.6 0;
    11.8 -0.2 0;
    16.8 1.5 0;
    21.1 5.6 0;
    25.1 11.7 0;
    30.1 12.1 0;
    32.6 10.3 0;
    34.4 6.1 0;
    34.8 0.9 0;
    33 -0.9 0;
    30.4 -1.4 0;
    25.7 -1.2 0;
    20.9 -1.8 0;
    18.1 -4.7 0;
    16.3 -8.6 0;
    17.5 -13.7 0;
    20.9 -16.1 0;
    25.7 -18.4 0;
    29.4 -19.6 0];
speed = 32;
trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [4 23.3 0]);
waypoints = [4 23.3 0;
    7 23.3 0;
    14.3 23.2 0;
    20.2 19.1 0;
    16.6 12.2 0;
    9 10.1 0;
    4.3 8.2 0;
    5.1 3.2 0;
    10.2 3 0;
    17.9 6 0;
    22.4 13.3 0;
    27.5 14.2 0;
    33.4 13.8 0;
    35.3 8 0;
    37.8 0.2 0;
    33.2 -3.1 0;
    25.7 -4.5 0;
    20.3 -6.8 0;
    23 -16.2 0;
    29.9 -16.1 0];
speed = 31;
trajectory(car1, waypoints, speed);

