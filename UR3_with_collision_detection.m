function MoveToTargetPosition(robotArm, targetPos, jointLimits, envObstacles)
    disp('Calculating target joint configuration...');
    approachPos = [targetPos(1), targetPos(2), targetPos(3)];
    downwardOrientation = rpy2tr(pi, 0, 0);
    approachTransform = transl(approachPos) * downwardOrientation;

    initialConfig = robotArm.model.getpos();
    targetConfig = robotArm.model.ikcon(approachTransform, initialConfig);

    for i = 1:length(targetConfig)
        targetConfig(i) = min(max(targetConfig(i), jointLimits(i, 1)), jointLimits(i, 2));
    end

    steps = 100; 
    jointPath = jtraj(initialConfig, targetConfig, steps);

    % Animate movement with collision checking
    for i = 1:steps
        robotArm.model.animate(jointPath(i, :));
        drawnow();

        % Check for collision (if any obstacles are provided)
        collisionDetected = false;
        for obs = envObstacles
            if obs.type == "ellipsoid"
                collisionDetected = checkEllipsoidCollision(robotArm, jointPath(i, :), obs);
            elseif obs.type == "plane"
                collisionDetected = checkPlaneCollision(robotArm, jointPath(i, :), obs.faces, obs.vertex, obs.faceNormals);
            end
            if collisionDetected
                disp(['Collision detected with ', obs.type, ' obstacle! Halting robot.']);
                break;
            end
        end

        if collisionDetected
            disp('Collision detected, stopping movement.');
            break;
        end
    end

    if ~collisionDetected
        disp('Robot reached target position without collision.');
    end
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
        % Extract the start and end points of each link
        linkStart = linkPoses(:, :, linkIdx);
        startP = linkStart(1:3, 4)';
        linkEnd = linkPoses(:, :, linkIdx + 1);
        endP = linkEnd(1:3, 4)';

        % Check intersection with each face of the obstacle
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

%robotArm = UR3(transl(0, 0, 0));
%MoveToTargetPosition(robotArm, [0.3, 0.3, 0.3], robotArm.model.qlim, []);

robotArm = UR3(transl(0, 0, 0));

% Define a box obstacle in the path
obstacleCenter = [0.1, -0.1, 0.05];  % Adjust position to be in the arm's path
sideLength = 0.05;  % Size of the box
[faces, vertices] = createObstacleBox(obstacleCenter, sideLength);
faceNormals = computeFaceNormals(faces, vertices);

% Define the obstacle struct
boxObstacle.type = "plane";
boxObstacle.faces = faces;
boxObstacle.vertex = vertices;
boxObstacle.faceNormals = faceNormals;

% Add the obstacle to the environment
envObstacles = [boxObstacle];


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
   % robotArm = UR3();
   % MoveToTargetPosition(robotArm, [0.3, 0.3, 0.3], robotArm.model.qlim, envObstacles);

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
    collisionDetected = false;
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

