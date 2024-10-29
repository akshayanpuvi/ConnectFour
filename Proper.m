function MoveToTargetPosition(robotArm, targetPos, jointLimits, envObstacles)
    disp('Calculating target joint configuration...');
    
    % Set target position and orientation
    approachPos = [targetPos(1), targetPos(2), targetPos(3)];
    downwardOrientation = rpy2tr(pi, 0, 0);
    approachTransform = transl(approachPos) * downwardOrientation;

    % Get the initial configuration of the robot
    initialConfig = robotArm.model.getpos();
    targetConfig = robotArm.model.ikcon(approachTransform, initialConfig);

    % Ensure target configuration is within joint limits
    for i = 1:length(targetConfig)
        targetConfig(i) = min(max(targetConfig(i), jointLimits(i, 1)), jointLimits(i, 2));
    end

    % Generate trajectory with fine interpolation for accurate collision detection
    jointPath = InterpolateWaypointRadians([initialConfig; targetConfig], deg2rad(1));

    % Animate movement with collision checking
    collisionDetected = false;  % Initialize variable
    for i = 1:size(jointPath, 1)
        robotArm.model.animate(jointPath(i, :));
        drawnow();

        % Check for collision at each step
        collisionDetected = IsCollision(robotArm, jointPath(i, :), envObstacles);
        if collisionDetected
            disp('Collision detected, stopping movement.');
            break;
        end
    end

    if ~collisionDetected
        disp('Robot reached target position without collision.');
    end
end

% Checks for collision along the robot's path
function result = IsCollision(robotArm, jointConfig, envObstacles)
    result = false;

    % Check if envObstacles is empty or has no faces
    if isempty(envObstacles) || ~isfield(envObstacles, 'faces')
        return; % No obstacles to check, so assume no collision
    end

    % Get the transform of every joint (i.e., start and end of every link)
    tr = GetLinkPoses(jointConfig, robotArm); % Using a single configuration here

    % Go through each link and check for collision with each face
    for i = 1:size(tr, 3)-1
        for faceIndex = 1:size(envObstacles.faces, 1)
            vertOnPlane = envObstacles.vertex(envObstacles.faces(faceIndex, 1)', :);
            [intersectP, check] = LinePlaneIntersection(envObstacles.faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i + 1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP, envObstacles.vertex(envObstacles.faces(faceIndex, :)', :))
                plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                display('Intersection');
                result = true;
                return;
            end
        end
    end
end

% Additional helper functions remain the same, such as `GetLinkPoses`, `InterpolateWaypointRadians`, and `LinePlaneIntersection`.


% Determine if a point is inside a triangle
function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);
    D = uv * uv - uu * vv;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)
        result = 0;
        return;
    end
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)
        result = 0;
        return;
    end
    result = 1;
end

% Line-plane intersection for collision checking
function [intersectionPoint, doesIntersect] = LinePlaneIntersection(planeNormal, pointOnPlane, lineStart, lineEnd)
    lineDir = lineEnd - lineStart;
    denom = dot(planeNormal, lineDir);
    if abs(denom) < 1e-6
        intersectionPoint = [NaN, NaN, NaN];
        doesIntersect = false;
        return;
    end
    t = dot(planeNormal, (pointOnPlane - lineStart)) / denom;
    if t < 0 || t > 1
        intersectionPoint = [NaN, NaN, NaN];
        doesIntersect = false;
    else
        intersectionPoint = lineStart + t * lineDir;
        doesIntersect = true;
    end
end

% Retrieve all link transforms for a given joint configuration
function linkTransforms = GetLinkPoses(q, robotArm)
    links = robotArm.model.links;
    linkTransforms = zeros(4, 4, length(links) + 1);
    linkTransforms(:,:,1) = robotArm.model.base;

    % Calculate each link's transformation
    for i = 1:length(links)
        L = links(1,i);
        currentTransform = linkTransforms(:,:,i);
        currentTransform = currentTransform * trotz(q(1,i) + L.offset) * ...
            transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        linkTransforms(:,:,i + 1) = currentTransform;
    end
end

% Interpolation Helper Functions

% Interpolates waypoints with smaller steps for collision detection accuracy
function qMatrix = InterpolateWaypointRadians(waypoints, maxStepRadians)
    qMatrix = [];
    for i = 1:size(waypoints, 1) - 1
        qMatrix = [qMatrix; FineInterpolation(waypoints(i, :), waypoints(i + 1, :), maxStepRadians)];
    end
end

% Fine-tune the interpolation to stay within max step size in radians
function qMatrix = FineInterpolation(q1, q2, maxStepRadians)
    steps = ceil(max(abs(q2 - q1)) / maxStepRadians);
    qMatrix = jtraj(q1, q2, steps);
end



% Check collision with ellipsoidal obstacles
function collisionDetected = checkEllipsoidCollision(robotArm, q, ellipsoid)
    linkPoses = getLinkTransforms(robotArm, q);
    collisionDetected = false;

    for linkIdx = 1:size(linkPoses, 3) - 1
        linkPose = linkPoses(:, :, linkIdx);
        distToEllipsoid = calculateAlgebraicDistance(linkPose(1:3, 4)', ellipsoid.center, ellipsoid.radii);
        if distToEllipsoid < 1
            collisionDetected = true;
            return;
        end
    end
end


% Check collision with plane obstacles (e.g., boxes)
function collisionDetected = checkPlaneCollision(robotArm, q, faces, vertex, faceNormals)
    linkPoses = getLinkTransforms(robotArm, q);
    collisionDetected = false;

    for linkIdx = 1:size(linkPoses, 3) - 1
        startP = linkPoses(1:3, 4, linkIdx)';
        endP = linkPoses(1:3, 4, linkIdx + 1)';

        for faceIdx = 1:size(faces, 1)
            faceVerts = vertex(faces(faceIdx, :), :);
            faceNorm = faceNormals(faceIdx, :);
            [intersection, doesIntersect] = computeLinePlaneIntersection(faceNorm, faceVerts(1, :), startP, endP);

            if doesIntersect && pointWithinFace(intersection, faceVerts)
                collisionDetected = true;
                return;
            end
        end
    end
end


% Obtain link transformations based on joint configuration
function linkTransforms = getLinkTransforms(robotArm, q)
    linkTransforms = repmat(eye(4), 1, 1, robotArm.model.n + 1);
    transform = robotArm.model.base;

    % Calculate each link's transformation
    for i = 1:robotArm.model.n
        transform = transform * robotArm.model.links(i).A(q(i));
        linkTransforms(:, :, i + 1) = transform;
    end
end

% Define a basic box obstacle with center and size
function [faces, vertices] = createObstacleBox(center, sideLength)
    halfSide = sideLength / 2;
    
    % Vertex coordinates of the box
    vertices = [
        center + [-halfSide, -halfSide, -halfSide];
        center + [halfSide, -halfSide, -halfSide];
        center + [halfSide, halfSide, -halfSide];
        center + [-halfSide, halfSide, -halfSide];
        center + [-halfSide, -halfSide, halfSide];
        center + [halfSide, -halfSide, halfSide];
        center + [halfSide, halfSide, halfSide];
        center + [-halfSide, halfSide, halfSide];
    ];
    
    % Face definitions using vertex indices
    faces = [
        1, 2, 3, 4;
        5, 6, 7, 8;
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
    ];
end

% Compute normals for each face of the obstacle
function faceNormals = computeFaceNormals(faces, vertices)
    faceNormals = zeros(size(faces, 1), 3);
    for i = 1:size(faces, 1)
        v1 = vertices(faces(i, 1), :);
        v2 = vertices(faces(i, 2), :);
        v3 = vertices(faces(i, 3), :);
        edge1 = v2 - v1;
        edge2 = v3 - v1;
        normal = cross(edge1, edge2);
        faceNormals(i, :) = normal / norm(normal);
    end
end

% Calculate algebraic distance to an ellipsoid
function algebraicDist = calculateAlgebraicDistance(point, center, radii)
    normalizedPoint = (point - center) ./ radii;
    algebraicDist = sum(normalizedPoint .^ 2);
end

% Calculate intersection point between line and plane
function [intersectionPoint, intersects] = computeLinePlaneIntersection(planeNormal, pointOnPlane, lineStart, lineEnd)
    lineDir = lineEnd - lineStart;
    denom = dot(planeNormal, lineDir);

    if abs(denom) < 1e-6
        intersectionPoint = [NaN, NaN, NaN];
        intersects = false;
        return;
    end

    t = dot(planeNormal, (pointOnPlane - lineStart)) / denom;
    if t < 0 || t > 1
        intersectionPoint = [NaN, NaN, NaN];
        intersects = false;
    else
        intersectionPoint = lineStart + t * lineDir;
        intersects = true;
    end
end

% Check if a point is inside a polygon face
function inside = pointWithinFace(point, faceVertices)
    numVertices = size(faceVertices, 1);
    normal = cross(faceVertices(2, :) - faceVertices(1, :), faceVertices(3, :) - faceVertices(1, :));
    normal = normal / norm(normal);

    [~, dropIdx] = min(abs(normal));
    projPoint = point; projVerts = faceVertices;
    projPoint(dropIdx) = []; projVerts(:, dropIdx) = [];

    inside = false;
    j = numVertices;
    for i = 1:numVertices
        if ((projVerts(i, 2) > projPoint(2)) ~= (projVerts(j, 2) > projPoint(2))) && ...
                (projPoint(1) < (projVerts(j, 1) - projVerts(i, 1)) * ...
                (projPoint(2) - projVerts(i, 2)) / (projVerts(j, 2) - projVerts(i, 2)) + projVerts(i, 1))
            inside = ~inside;
        end
        j = i;
    end
end

function objectHandle = PlaceFlatObject(position)
    % PlaceFlatObject places the 'flatc4.ply' object at the specified position.
    %
    % Syntax:
    %   objectHandle = PlaceFlatObject(position)
    %
    % Inputs:
    %   position - A 1x3 vector specifying the [x, y, z] coordinates where the object should be placed.
    %
    % Outputs:
    %   objectHandle - Handle to the placed object.
    %
    % Example:
    %   % Place the object at coordinates [-0.5, 0.45, 0]
    %   desiredPosition = [-0.5, 0.45, 0];
    %   objectHandle = PlaceFlatObject(desiredPosition);

    % Load the object at the origin
    objectHandle = PlaceObject('flatc4.ply', [0, 0, 0]);

    % Get the vertices of the object
    vertObject = get(objectHandle, 'Vertices');

    % Scale the object
    scalingFactor = 0.05;  % Adjust the scaling factor if needed
    vertObject = vertObject * scalingFactor;

    % Rotate the scaled object
    rotationMatrix = trotx(pi/2);
    vertObject = (rotationMatrix(1:3, 1:3) * vertObject')';

    % Translate the object to the desired position
    vertObject(:, 1) = vertObject(:, 1) + position(1);
    vertObject(:, 2) = vertObject(:, 2) + position(2);
    vertObject(:, 3) = vertObject(:, 3) + position(3);

    % Update the object's vertices
    set(objectHandle, 'Vertices', vertObject);
end

function slotPositions = calculateSlotPositions(bottomLeft, topRight, numCols, numRows)
    slotPositions = zeros(numCols, numRows, 3);  % Initialize [x, y, z] coordinates for each slot
    
    % Calculate spacing between slots
    xSpacing = (topRight(1) - bottomLeft(1)) / (numCols - 1);
    ySpacing = (topRight(2) - bottomLeft(2)) / (numRows - 1);

    % Compute coordinates for each slot
    for col = 1:numCols
        for row = 1:numRows
            % Calculate position for each slot based on column, row, and spacing
            slotPositions(col, row, :) = [bottomLeft(1) + (col - 1) * xSpacing, ...
                                          bottomLeft(2) + (row - 1) * ySpacing, ...
                                          bottomLeft(3)];
        end
    end
end

function HoverOverSlot(robotArm, slotPosition, hoverHeight)
    % Add the hover height to position
    targetPosition = slotPosition + [0, 0, hoverHeight];
    disp(['Moving to hover over slot at position: ', mat2str(targetPosition)]);
    % Command the robot arm to hover over the slot
    MoveToTargetPosition(robotArm, targetPosition, robotArm.model.qlim, []);
end

function plotBoxObstacle(faces, vertices)
    % Function to plot a box obstacle in the current figure
    hold on;  % Retain the current plot for adding the obstacle
    axis equal;
    for i = 1:size(faces, 1)
        % Extract the vertices for the current face
        faceVertices = vertices(faces(i, :), :);
        
        % Plot the face using the patch function
        patch('Vertices', faceVertices, 'Faces', 1:4, ...
              'FaceColor', [1, 0, 0], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end

function newVertices = moveObstacle(vertices, newPosition)
    % Move obstacle vertices to a new position
    % Inputs:
    %   vertices - Original vertices of the obstacle
    %   newPosition - Desired [x, y, z] position for the obstacle center
    % Outputs:
    %   newVertices - Translated vertices of the obstacle

    % Calculate current center of the obstacle
    currentCenter = mean(vertices, 1);
    
    % Calculate the translation vector
    translation = newPosition - currentCenter;
    
    % Apply translation to all vertices
    newVertices = vertices + translation;
end

function countery = PlaceCoinYellow(position)

    % Load the coin object at the origin (no initial translation)
    countery = PlaceObject('ImageToStl.com_countery.ply', [0, 0, 0]);

    % Get the vertices of the coin object
    vertcountery = get(countery, 'Vertices');

    % Scale the coin object
    scalingFactor = 0.0005;  % Adjust as needed
    vertcountery = vertcountery * scalingFactor;

    % Translate the scaled coin to the desired position
    vertcountery(:, 1) = vertcountery(:, 1) + position(1);
    vertcountery(:, 2) = vertcountery(:, 2) + position(2);
    vertcountery(:, 3) = vertcountery(:, 3) + position(3);

    % Update the coin object's vertices
    set(countery, 'Vertices', vertcountery);
end

function countery = PlaceCoinRed(position)

    % Load the coin object at the origin (no initial translation)
    countery = PlaceObject('ImageToStl.com_counter.ply', [0, 0, 0]);

    % Get the vertices of the coin object
    vertcountery = get(countery, 'Vertices');

    % Scale the coin object
    scalingFactor = 0.0005;  % Adjust as needed
    vertcountery = vertcountery * scalingFactor;

    % Translate the scaled coin to the desired position
    vertcountery(:, 1) = vertcountery(:, 1) + position(1);
    vertcountery(:, 2) = vertcountery(:, 2) + position(2);
    vertcountery(:, 3) = vertcountery(:, 3) + position(3);

    % Update the coin object's vertices
    set(countery, 'Vertices', vertcountery);
end

function coinHandles = SpawnCoins(startPosition, numCoins, spacing, color)    
    coinHandles = gobjects(numCoins, 1);  % Initialize array for coin handles
    
    for i = 1:numCoins
        % Calculate position for each coin with the given spacing
        coinPosition = startPosition + [(i - 1) * spacing, 0, 0];
        
        % Spawn the coin based on color
        if strcmp(color, 'yellow')
            coinHandles(i) = PlaceCoinYellow(coinPosition);
        elseif strcmp(color, 'red')
            coinHandles(i) = PlaceCoinRed(coinPosition);
        else
            error('Invalid color specified. Choose either ''yellow'' or ''red''.');
        end
    end
end

%
% Main section with all the arms and game loaded in

% Set Up

% Arm placement
UR3_Arm = [0.001, -0.35, 0];
TM5_Arm = [0.001, 0.4, 0];



% Board Placement
Board = PlaceFlatObject([0, 0, 0]);
yellowStartPos = [-0.2, -0.5, 0.005];
redStartPos = [0.3, 0.5, 0.005];
% Spawn yellow and red coins near each arm


% Define board boundaries and number of slots
bottomLeft = [-0.3, -0.25, 0];   
topRight = [0.3, 0.25, 0];       
numCols = 7;                       
numRows = 6;                        

slotPositions = calculateSlotPositions(bottomLeft, topRight, numCols, numRows); %calculates each slot position


Arm_1 = UR3(transl(UR3_Arm(1), UR3_Arm(2), UR3_Arm(3)));
Arm_2 = LinearTM5(transl(TM5_Arm(1), TM5_Arm(2), TM5_Arm(3)));

s = 2;
hoverHeight = 0.1;



while true  % Infinite loop for alternating turns
    % Prompt user for column and row input
    column = input('Enter the column number (or -1 to stop): ');
    if column == -1
        disp('Stopping the game.');
        break;  % Exit the loop if the user enters -1
    end

    row = input('Enter the row number (or -1 to stop): ');
    if row == -1
        disp('Stopping the game.');
        break;  % Exit the loop if the user enters -1
    end
    % Ensure valid column and row input
    if column >= 1 && column <= numCols && row >= 1 && row <= numRows
        hoverPosition = squeeze(slotPositions(column, row, :))';  % Extract slot position

        % Alternate turns between Arm_1 and Arm_2
        if s == 1
            MoveToTargetPosition(Arm_1, yellowStartPos , Arm_1.model.qlim, []);
            MoveToTargetPosition(Arm_1, [yellowStartPos(1), yellowStartPos(2), 0.2] , Arm_1.model.qlim, []);
            HoverOverSlot(Arm_1, hoverPosition, 0.2);
            HoverOverSlot(Arm_1, hoverPosition, hoverHeight);
            HoverOverSlot(Arm_1, hoverPosition, 0.2);
            MoveToTargetPosition(Arm_1, [yellowStartPos(1), yellowStartPos(2), 0.2] , Arm_1.model.qlim, []);
            s = 2;  % Switch to Arm_2 for the next turn
        else
            MoveToTargetPosition(Arm_2, [-0.6, 0.5, 0.5] , Arm_2.model.qlim, []);  
            HoverOverSlot(Arm_2, hoverPosition, 0.4);
            HoverOverSlot(Arm_2, hoverPosition, hoverHeight);
            HoverOverSlot(Arm_2, hoverPosition, 0.4);
            MoveToTargetPosition(Arm_2, [-0.6, 0.5, 0.5] , Arm_2.model.qlim, []); 
            s = 1;  % Switch to Arm_1 for the next turn
        end
    else
        disp('Invalid column or row number. Please enter values within the slot grid range.');
    end
end


%%
%robotArm = LinearTM5;
%destination = [0.2, 0.2, 0.2];
%MoveToTargetPosition(robotArm, destination, robotArm.model.qlim, []);
% Main script
figure;
axis([-1 1 -1 1 0 1]);  % Set axis limits to ensure all elements are visible
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
hold on;

% Initialize the robot arm at the origin
% Initialize robot and environment
robotArm = UR3();  % Replace with appropriate initialization if needed

% Define joint limits for your robot
% Define an obstacle in the environment
[faces, vertices] = createObstacleBox([0.3, 0, 0.3], 0.2);  % Center at [0.3, 0, 0.3] with side length 0.5
faceNormals = computeFaceNormals(faces, vertices);

% Create obstacle struct to pass to function
obstacle.type = 'plane';
obstacle.faces = faces;
obstacle.vertex = vertices;
obstacle.faceNormals = faceNormals;
envObstacles = [obstacle];

% Plot the obstacle for visualization
plotBoxObstacle(faces, vertices);

% Define target position within or just behind obstacle to test collision
targetPos = [0.3, 0, 0.3];  % Adjust based on the position of the obstacle

% Test MoveToTargetPosition with collision detection
MoveToTargetPosition(robotArm, [0.3, 0.2, 0.3] , robotArm.model.qlim, envObstacles);
MoveToTargetPosition(robotArm, [0.3, -0.2, 0.3] , robotArm.model.qlim, envObstacles);



%% This script allows the arm to hover over any slot you want. 

% Define board boundaries and number of slots
bottomLeft = [-0.3, -0.25, 0];     % Bottom-left corner of the Connect 4 board
topRight = [0.3, 0.25, 0];         % Top-right corner of the Connect 4 board
numCols = 7;                        % Number of columns in Connect 4
numRows = 6;                        % Number of rows in Connect 4

% Initialize Connect 4 board and UR3 arm
Board = PlaceFlatObject([0, 0, 0]);
robotArm = UR3(transl(0.001, -0.35, 0));  % Adjust initial position as needed

% Calculate slot positions for Connect 4 board
slotPositions = calculateSlotPositions(bottomLeft, topRight, numCols, numRows);

% Define the hover height above the board
hoverHeight = 0.1;  % Adjust hover height as needed

% Example: Hover over slot in column 4, row 2
column = 3;
row = 5;
hoverPosition = squeeze(slotPositions(column, row, :))';  % Extract the position for the selected slot
HoverOverSlot(robotArm, hoverPosition, hoverHeight);      % Command arm to hover over this slot


%%

% Define initial robot setup with a fixed base
robotArm = UR3();  % Initialize UR3 with base at origin

% Define the start, end positions for the end effector, and obstacles
startPos = [0.2, -0.2, 0.2];      % Starting position of the end effector
endPos = [0.6, 0.6, 0.6];         % Target position for the end effector
jointLimits = robotArm.model.qlim; % Joint limits from the UR3 model

% Define obstacle properties (a box in the path)
[faces, vertices] = createObstacleBox([0.4, 0.1, 0.3], 0.15); % Box at center with side 0.15m
obstacle.faces = faces;
obstacle.vertex = vertices;
obstacle.faceNormals = computeFaceNormals(faces, vertices);
obstacle.type = "plane";

% Define the environment with obstacles
envObstacles = [obstacle];

% Initialize trajectory planning parameters
currentPos = startPos;
currentConfig = robotArm.model.getpos();  % Get initial joint configuration

% Loop to plan and execute trajectory until target is reached
while norm(currentPos - endPos) > 0.05  % Stop if close enough to target
    % Calculate target pose and trajectory
    approachTransform = transl(endPos) * rpy2tr(pi, 0, 0); % Target pose with end-effector down
    targetConfig = robotArm.model.ikcon(approachTransform, currentConfig);

    % Generate joint trajectory
    steps = 50;
    qMatrix = jtraj(currentConfig, targetConfig, steps);

    % Execute the trajectory with collision detection
    chfalse;
    for i = 1:steps
        robotArm.model.animate(qMatrix(i, :));
        drawnow;
        
        % Update current end-effector position
        currentPos = robotArm.model.fkine(qMatrix(i, :)).t';
        
        % Check for collisions at each step
        for obs = envObstacles
            if obs.type == "plane"
                collisionDetected = checkPlaneCollision(robotArm, qMatrix(i, :), obs.faces, obs.vertex, obs.faceNormals);
            end
            if collisionDetected
                disp('Collision detected! Recalculating path...');
                break;
            end
        end

        % If collision is detected, adjust target position to avoid obstacle
        if collisionDetected
            endPos = endPos + [0, 0.05, 0.05];  % Offset to move around obstacle
            break;
        end
    end

    % Update the current configuration if path was collision-free
    if ~collisionDetected
        currentConfig = targetConfig;
    end
end

disp('Path completed without collision');



%%
% Define ellipsoid obstacle with all necessary fields
ellipsoidObs.type = "ellipsoid";
ellipsoidObs.center = [0.5, 0.2, 0.3];
ellipsoidObs.radii = [0.2, 0.2, 0.3];
ellipsoidObs.faces = [];        % Unused, set to empty
ellipsoidObs.vertex = [];       % Unused, set to empty
ellipsoidObs.faceNormals = [];   % Unused, set to empty

% Define a simple example plane-based obstacle (box) with placeholder faces and vertices
planeObs.type = "plane";
planeObs.center = [];           % Unused, set to empty
planeObs.radii = [];            % Unused, set to empty
planeObs.faces = [1, 2, 3, 4;   % Example faces
                  5, 6, 7, 8];
planeObs.vertex = [0, 0, 0;     % Example vertices (define a simple box shape)
                   1, 0, 0;
                   1, 1, 0;
                   0, 1, 0;
                   0, 0, 1;
                   1, 0, 1;
                   1, 1, 1;
                   0, 1, 1];
planeObs.faceNormals = [0, 0, 1;   % Example normals for each face
                        0, 0, -1];

% Combine obstacles into an array with matching fields
envObstacles = [ellipsoidObs, planeObs];


robotArm = UR3(transl(0, 0, 0));
MoveToTargetPosition(robotArm, [0.3, 0.3, 0.3], robotArm.model.qlim, envObstacles);
