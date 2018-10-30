%Abhinav Agrahari 
%-- includes overpass roads
%Main part of challenge is to clone the repo, create a branch on the repo, 
%come up with own road simulation, own sensor configuration
%then create a pull request so they can see it
%2nd part of the challenge is to understand everything, to understand how
%the object detection works etc.

%our first delivarable is due mid nov, then jan, then march

% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;

%Defining the complex road below
roadCenters = [54.7 42.01 0;
    99.8 42 0];
%the following marking gives a double solid yellow in the middle of 2 lanes
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Solid')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);
%Below is the loop de loop road that acts as an overpass
roadCenters = [98 48.8 0;
    102.7 80.2 0;
    118.5 93.7 8; %8 in the last coloumn means this road center is elevated 8 up
    154 81.4 0;
    149.3 37.1 8; %the other overpass point
    123.1 4.2 0;
    100.7 35.9 0;
    98 48.8 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Solid')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [148.6 -21.2 0;
    121.4 91.9 0;
    59.5 82.5 0;
    57.9 39.7 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(4, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [153.6 81.9 0;
    198 92 0;
    215.3 86.6 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Unmarked')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% ================ VEHICLES ==========
% Add the ego car
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [153.8 -19.1 0]);
%The third coloumn in the way point below denotes elevation!
waypoints = [153.8 -19.1 0;
    154.9 -4.42 0;
    156.9 11.98 0;
    147.11 67.7 0.01;
    144.2 72.68 0;
    133.1 87.8 0;
    130.1 91.58 0;
    117.1 100.3 0;
    104.4 104.2 0;
    93.2 107.18 0;
    83.6 104.9 0;
    74.5 101.5 0;
    66 97.2 0;
    58 88.7 0;
    51.2 73.9 0;
    50 59.3 0;
    50.4 48.58 0;
    55.6 41.6 0;
    62.8 40.2 0;
    79.3 40.6 0;
    98 37.6 0;
    99.4 32.38 0;
    102 22.48 0;
    105.6 13.88 0;
    113.2 5.8 0;
    123.8 1.6 0;
    131.5 4.1 0.6;
    137 7.3 1.8;% the car is travelling on the overpass at this point
    144.5 18.1 4.8;
    150 32.4 7.8;
    152.3 38.8 7.9;
    155.8 48.6 6.6;
    158.7 59.5 3.8;
    159.5 70.2 1.4;
    157.9 78.2 0.2;
    156.4 81.3 0.1;
    161.9 83.4 0;
    171.3 86.4 0; 
    181.7 90 0;
    188.26 90.15 0.01;
    199.8 90 0;
    210.6 86.8 0];
speed = 25;
trajectory(egoCar, waypoints, speed);
% Instead of path(passingCar, waypoints, 35), we can also use trajectory()
%Note: path is depcrecated:
%See: https://www.mathworks.com/help/driving/ref/path.html

% Add the other actors
truck = vehicle(scenario, ... % this is a truck that starts on the oppisite side
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [208.21 91.17 0.01]);
waypoints = [208.21 91.17 0.01;
    201.16 93.33 0.01;
    187.56 94.56 0.01;
    171.79 90.26 0.01;
    157.42 85.91 0.08;
    154.64 84.31 0.11;
    149.19 90.22 1.18;
    142.9 95.1 3.2;
    136.5 97.12 4.92;
    128.2 97.6 6.9;
    115.2 94.6 7.6;
    102.9 83.9 0.4;
    96 68.8 0;
    96.3 55.2 0;
    96.3 45.3 0;
    79 44.5 0;
    59.9 44.4 0;
    58.5 55 0;
    57.6 69.7 0;
    68.5 88.7 0;
    85.8 96.9 0;
    103.4 96.1 0;
    124.4 87.6 0;
    130.4 77.6 0;
    142 58.7 0;
    145.3 43.8 0;
    149.4 9.4 0;
    144.5 -14.2 0];
speed = 16; %trucks are slow
trajectory(truck, waypoints, speed);

%The following car recklessly drives around the track pretty fast
% and starts on the oppisite side but loops around, and speeds past the ego
% car
crazyCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [57 62.8 0]);
waypoints = [57.8 64.3 0;
    61.3 77.7 0;
    71.2 91 0;
    90.2 100.8 0;
    108.7 95.6 0;
    123.9 86.1 0;
    141.8 67.1 0;
    148.6 50.6 0;
    147.2 59.8 0;
    135.3 84.1 0;
    112.2 102 0;
    89.1 104.9 0;
    63.3 94.9 0;
    52.1 57.7 0;
    63.8 44.7 0;
    101.5 41.2 0;
    108.5 17.3 0;
    117.9 8.1 0;
    129.9 7.8 0.6;
    139.9 17.8 3.7;
    147.7 36.2 8;
    156.9 58.9 4.3;
    154.1 73.4 1;
    153.06 81.08 0.09;
    178.1 88.8 0;
    191.3 91.3 0;
    197.7 93.2 0;
    195.9 93.8 0;
    168.1 87.3 0;
    161.22 86.25 0.01;
    149.42 85.62 0.47;
    137.5 93.6 4.4;
    119.2 92.42 7.94;
    107.38 85.56 1.85;
    99.7 69.6 0;
    99.9 48.5 0;
    97 42.2 0;
    92.7 39.9 0;
    75.7 42.1 0;
    75.7 42.1 0;
    75.7 42.1 0];
speed = 38;
trajectory(crazyCar, waypoints, speed);

sensors = cell(8,1); % have to make cell instead of array because different sensors
%front camera, mounted on rear-view mirror
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ... 
    'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], ...
    'MaxRange', 100, ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]));
   %The following parameter was causing the simulation to not run: 'DetectorOutput', 'Lanes and objects');
   % not sure why, ---> note to self: look into it later
   %Doesn't seem to work with 'objects only' either.

sensors{2} = visionDetectionGenerator('SensorIndex', 2, ... %rear camera
    'SensorLocation', [0.2*egoCar.Wheelbase 0], ...
    'FalsePositivesPerImage', 0.1, ...
    'Height', 1.1, ...
    'Yaw', 180, ...
    'MaxRange', 100, ...
    'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]));

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 128, ... %'sensorindex', snesorIndexNum, 'height' etc.
    'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 50, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
%'SensorLocation', [frontBackPosition, leftright position
% field of view is ??????
% azimuth = 10, the angle between two objects that the car can distinguish. 10 is recommended

%Rear radars have a greater range to detect cars behind ego car at a safer
%distance
% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -128, ...
    'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 50, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-right-facing radar sensor at the right front wheel well of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], ...
    'Yaw', -56.8, ...
    'MaxRange', 35, ...
    'FieldOfView', [90 5],...
    'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

%Front-left-facing radar sensor at the left front wheel well of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], ...
    'Yaw', 56.8, ...
    'MaxRange', 35, ...
    'FieldOfView', [90 5],...
    'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);
% yaw is degree from front of car (horizontally)

sensors{7} = radarDetectionGenerator('SensorIndex', 7, ...
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang 0], ...
    'MaxRange', 140,...
    'FieldOfView', [90 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
%Radar narrower than the rear camera.
sensors{8} = radarDetectionGenerator('SensorIndex', 8, ...
    'SensorLocation', [-egoCar.RearOverhang, 0], ...
     'MaxRange', 140,...
    'Yaw', -180);
%Begin other stuff that we don't have to modify for this workshop:
% Initiate the tracker
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ... % @ is a function handler
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);%function from matlab

% sensor can't tell object moving through different frames
toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent) % passing the display we just created
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,8);
    for i = 1:8 %for loop for the 8 sensors, to simulate data the sensor will see
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
        detectionClusters = clusterDetections(detections, vehicleLength); % we can tell if we detected an object because there will be  cluster of dots
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
%tries to find a cluster size that is approximately the size of a vehicle
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
%step 11
%
function BEP = createDemoDisplay(egoCar, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

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
    for i = 7:8
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
%if new cluster, then make a new track
% if same car as last frame, then just keep in its tracker
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