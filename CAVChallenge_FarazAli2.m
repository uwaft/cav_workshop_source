function [allData, scenario, sensors] = CAVChallenge_FarazAli2()
%CAVChallenge_FarazAli - Returns sensor detections
%    allData = CAVChallenge_FarazAli returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = CAVChallenge_FarazAli optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.5 and Automated Driving System Toolbox 1.3.
% Generated on: 28-Nov-2018 19:52:00

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
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.9 0], ...
    'DetectorOutput', 'Objects only', ...
    'ActorProfiles', profiles);
sensors{2} = radarDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -41.7689900996318, ...
    'MaxRange', 50, ...
    'FieldOfView', [90 5], ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'DetectorOutput', 'Objects only', ...
    'ActorProfiles', profiles);
sensors{4} = radarDetectionGenerator('SensorIndex', 4, ...
    'SensorLocation', [2.8 0.9], ...
    'Yaw', 42.0390638658363, ...
    'MaxRange', 50, ...
    'FieldOfView', [90 5], ...
    'ActorProfiles', profiles);
sensors{5} = radarDetectionGenerator('SensorIndex', 5, ...
    'SensorLocation', [1.9 0], ...
    'Yaw', 93.9301755457255, ...
    'ActorProfiles', profiles);
sensors{6} = radarDetectionGenerator('SensorIndex', 6, ...
    'SensorLocation', [1.9 0], ...
    'Yaw', -92.3970285300596, ...
    'ActorProfiles', profiles);
sensors{7} = radarDetectionGenerator('SensorIndex', 7, ...
    'SensorLocation', [0 0.9], ...
    'Yaw', 131.771352130364, ...
    'MaxRange', 25, ...
    'FieldOfView', [90 5], ...
    'ActorProfiles', profiles);
sensors{8} = radarDetectionGenerator('SensorIndex', 8, ...
    'SensorLocation', [0 -0.9], ...
    'Yaw', -128.168994050648, ...
    'MaxRange', 25, ...
    'FieldOfView', [90 5], ...
    'ActorProfiles', profiles);
numSensors = 8;

function [scenario, egoCar] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [-7.50000000000001 -18.8 0;
    12.4 0.2 0;
    29.3 2.9 0;
    42.4 10.5 0;
    54.1 17.1 0;
    74.4 25.6 0;
    91.2 32 0;
    135.4 20.9 0;
    142.2 -32 0;
    105.2 -98 0;
    71.3 -130.8 0;
    8.3 -123.1 0;
    -19.1 -79.5 0;
    -7.50000000000001 -18.8 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(3, 'Width', [4.425 4.425 3.6], 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [133 -46.5 0;
    95.7 -58.7 0;
    66 -75.4 0;
    54.9 -88.5 0];
road(scenario, roadCenters);

roadCenters = [11.9 -71 0;
    29.2 -1.3 0];
road(scenario, roadCenters);

% Add the ego car
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-1.1 -11.7 0]);
waypoints = [-1.1 -11.7 0;
    2.6 -6.3 0;
    7.2 -2.7 0;
    12.9 -0.300000000000001 0;
    19.4 1 0;
    25 1.6 0;
    30.5 2.5 0;
    35.1 5 0;
    40.1 8.5 0;
    45.6 12.3 0;
    47.5 13.8 0;
    54 17 0;
    66.2 22.1 0;
    79.4 27.4 0;
    92.6 32.4 0;
    107.2 33.5 0;
    121.5 31.3 0;
    127.9 27.1 0;
    134.3 22.8 0;
    137.9 16.7 0;
    142.5 7.1 0;
    144.4 0.8 0;
    145.3 -11 0;
    140.8 -17.9 0;
    139.2 -25.9 0;
    136.3 -34.5 0;
    133.4 -43.2 0;
    130.4 -47.7 0;
    124.6 -48.9 0;
    115.2 -52 0;
    103.3 -56.3 0;
    90.2 -61 0;
    87.1 -62.9 0;
    78.9 -66.3 0;
    61.8 -80.4 0;
    55.4 -87.5 0];
speed = [60;60;60;60;60;60;60;60;60;60;60;60;60;60;60;60;60;60;80;80;80;80;80;80;60;60;60;60;60;60;60;60;60;60;60;60];
trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [50.5 19.5 0]);
waypoints = [50.5 19.5 0;
    41 14.3 0;
    36.5 11 0;
    30.1 7.7 0;
    23.2 5.6 0;
    15.3 4.3 0;
    8.2 2.8 0;
    2.2 -0.9 0;
    -1.2 -4.4 0;
    -10.2 -15.4 0;
    -16.9 -25.6 0;
    -19.1 -33.6 0;
    -24 -47.3 0;
    -24.9 -70.8 0;
    -21.8 -87.6 0;
    -13.4 -106.2 0;
    -1 -120 0;
    8.8 -128.8 0;
    19.7 -133.7 0;
    43.2 -141 0;
    54.1 -141 0;
    67.3 -136.9 0;
    85.4 -126.4 0;
    92 -121.2 0;
    122.1 -82.7 0;
    141.6 -48.6 0;
    143 -44.8 0];
speed = 40;
trajectory(car1, waypoints, speed);

truck = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [17.7 -47.4 0]);
waypoints = [17.7 -47.4 0;
    24.5 -19.3 0;
    27.2 -9.5 0;
    30 -1.8 0;
    35.4 0.9 0;
    39.9 3.3 0;
    46.6 7.8 0;
    56.2 12.6 0;
    70.2 19 0;
    84.7 25.3 0;
    97.3 28.4 0;
    108.7 28.8 0;
    121.6 25.4 0;
    127.1 22.4 0;
    133.8 15.3 0;
    137.8 9.2 0;
    140.7 -0.9 0;
    141.3 -10.7 0;
    140.7 -16.1 0;
    139.2 -23.8 0;
    137.4 -30.3 0;
    132.7 -44.1 0;
    129.9 -49 0;
    127.5 -56.3 0;
    116.6 -73.1 0;
    109.9 -84 0;
    102.3 -93.8 0];
speed = 60;
trajectory(truck, waypoints, speed);

