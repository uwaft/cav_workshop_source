% NOTE: Added a camera on the front right side of the (left-side-steering) car. Has 2 benefits:
% 1. Useful while turning right on a roundabout as shown in the scenerio
% (which can be a difficult maneuver if there's a passenger in the front
% seat)
% 2. Also useful in trucks and bigger vehicles which have that area as a
% blindspot.
% Remark: THe came would be added on the left side for a righ-side-steering
% vehicle.


% Define an empty scenario.
scenario = drivingScenario;
%scenario.SampleTime = 0.01;

%roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];
%road(scenario, roadCenters, 'lanes',lanespec(2));

% Add all road segments
roadCenters = [-10.9 -3.5 0;
    26.1 -3.2 0;
    50.9 -3.1 0];
laneSpecification = lanespec(2, 'Width', 8);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [25.6 29.3 0;
    26.2 -50.7 0];
laneSpecification = lanespec(2, 'Width', 8);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego car
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-8.1 -7.4 0]);
waypoints = [-8.1 -7.4 0;
    5.1 -7.2 0;
    9.5 -7 0;
    13.1 -6.9 0;
    15.6 -6.9 0;
    17.7 -6.7 0;
    20.5 -7.7 0;
    23 -9 0;
    25.1 -9.2 0;
    28.4 -8.3 0;
    31.3 -6.3 0;
    31.1 -0.5 0;
    29.2 1.5 0;
    29.2 4.5 0;
    29.4 8.3 0;
    29.5 13.3 0;
    29.5 19.9 0;
    29.7 26.8 0];
speed = 35;
trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [29.9 -48 0]);
waypoints = [29.9 -48 0;
    30.1 -29.9 0;
    30.1 -25.2 0;
    30.1 -18.4 0;
    29.9 -13.4 0;
    29.9 -11.4 0;
    31.3 -8.5 0;
    32 -3.2 0;
    30.9 1 0;
    29.5 2.6 0;
    29 5.1 0;
    29.4 9.2 0;
    29.5 14 0];
speed = 25;
trajectory(car1, waypoints, speed);

% Creating the roundabout using barriers
actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [25.5 -0.4 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [23.5 -1.7 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [29.2 -3.2 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [27.6 -1.8 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [23.5 -4.9 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [25.5 -6 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [27.8 -4.8 0]);

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [22.7 -3.2 0]);



sensors = cell(8,1);
% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ...
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
    'SensorLocation', [-egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5]);

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
    'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
% 
% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
    'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-facing camera located at front windshield.
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1);

% Right-side-facing camera located near the right back-view mirror.
sensors{9} = visionDetectionGenerator('SensorIndex', 9, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'Height', 0.2, 'Yaw', -60,  ... 
     'MaxRange', 30);

% Rear-facing camera located at rear windshield.
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180);

% Initiate the tracker
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);

toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,9);
    for i = 1:9
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
            end
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end

    % Update the tracker if there are new detections
    if any(isValidTime)
        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionClusters, time);

        % Update bird's-eye plot
        updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
    end

    % Snap a figure for the document when the car passes the ego vehicle
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end

function filter = initSimDemoFilter(detection)
    % Use a 2-D constant velocity model to initialize a trackingKF filter.
    % The state vector is [x;vx;y;vy]
    % The detection measurement vector is [x;y;vx;vy]
    % As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
    H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
    filter = trackingKF('MotionModel', '2D Constant Velocity', ...
        'State', H' * detection.Measurement, ...
        'MeasurementModel', H, ...
        'StateCovariance', H' * detection.MeasurementNoise * H, ...
        'MeasurementNoise', detection.MeasurementNoise);
end

function detectionClusters = clusterDetections(detections, vehicleSize)
    N = numel(detections);
    distances = zeros(N);
    for i = 1:N
        for j = i+1:N
            if detections{i}.SensorIndex == detections{j}.SensorIndex
                distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
            else
                distances(i,j) = inf;
            end
        end
    end
    leftToCheck = 1:N;
    i = 0;
    detectionClusters = cell(N,1);
    while ~isempty(leftToCheck)
        % Remove the detections that are in the same cluster as the one under
        % consideration
        underConsideration = leftToCheck(1);
        clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
        detInds = leftToCheck(clusterInds);
        clusterDets = [detections{detInds}];
        clusterMeas = [clusterDets.Measurement];
        meas = mean(clusterMeas, 2);
        meas2D = [meas(1:2);meas(4:5)];
        i = i + 1;
        detectionClusters{i} = detections{detInds(1)};
        detectionClusters{i}.Measurement = meas2D;
        leftToCheck(clusterInds) = [];
    end
    detectionClusters(i+1:end) = [];

    % Since the detections are now for clusters, modify the noise to represent
    % that they are of the whole car
    for i = 1:numel(detectionClusters)
        measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
        measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
        detectionClusters{i}.MeasurementNoise = measNoise;
    end
end

function BEP = createDemoDisplay(egoCar, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from left
    hLeftViewPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 0.5], 'Title', 'Left Camera View');
    hCarPlot = axes(hLeftViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot,'ViewHeight', 3, 'ViewLocation', [0 10], 'ViewYaw', -67);
    
    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

    
    
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0.5 0.5 0.5], 'Title', 'Bird''s-Eye Plot');

    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

    % Plot the coverage areas for radars
    for i = 1:6
        cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end

    % Plot the coverage areas for vision sensors
    for i = 7:9
        cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, 45);
    end

    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');

    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');

    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');

    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);

    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);

    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end

function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);

    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6 % Vision detections
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));

    % Prepare and update tracks display
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end
