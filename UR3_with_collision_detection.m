function MoveToTargetPosition(robotArm, targetPos, jointLimits, envObstacles)
    % Define the safe approach position based on targetPos
    approachPos = [targetPos(1), targetPos(2), targetPos(3)];
    downwardOrientation = rpy2tr(pi, 0, 0);
    approachTransform = transl(approachPos) * downwardOrientation;

    % Initial and target joint configurations
    initialConfig = robotArm.model.getpos();
    targetConfig = robotArm.model.ikcon(approachTransform, initialConfig);

    % Enforce joint limits
    for i = 1:length(targetConfig)
        targetConfig(i) = min(max(targetConfig(i), jointLimits(i, 1)), jointLimits(i, 2));
    end

    % Create a smooth trajectory
    steps = 100; 
    jointPath = jtraj(initialConfig, targetConfig, steps);

    % Animate movement with collision checking
    for i = 1:steps
        robotArm.model.animate(jointPath(i, :));
        drawnow();

        % Collision detection for each obstacle
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

% Define ellipsoid obstacle with all necessary fields
ellipsoidObs.type = "ellipsoid";
ellipsoidObs.center = [0.5, 0.2, 0.3];
ellipsoidObs.radii = [0.2, 0.2, 0.3];
ellipsoidObs.faces = [];        % Unused, set to empty
ellipsoidObs.vertex = [];       % Unused, set to empty
ellipsoidObs.faceNormals = [];   % Unused, set to empty

% Define a simple example box with placeholder faces and vertices
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
