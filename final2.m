% Clear the current figure before plotting the new bricks and robot
clf;

% Set a fixed axis range to prevent zooming or autoscaling
axis([-1 1 -1 1 -0.5 1]);  
hold on;  % Hold the plot so that subsequent objects don't overwrite the figure

% Disable autoscaling for the axes
ax = gca;  % Get current axes
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';

A = UR3;
jointLimits = A.model.qlim;

function MoveToApproachPositionWithCoin(A, initialPosition, jointLimits, coin)
    % Move the robot to a position directly above the brick (safe approach)
    % Returns qApproach (joint angles for the approach), qInitial (starting joint angles),
    % approachPose (target pose), and qMatrixApproach (trajectory) for later use
    % Define the position above the brick with the specified approach height
    approachPosition = [initialPosition(1), initialPosition(2), initialPosition(3)];

    % Set the end-effector orientation to face downward (roll = pi radians)
    targetOrientation = rpy2tr(pi, 0, 0);  % Roll = pi (180 degrees), end-effector points down

    % Combine the position and orientation into a homogeneous transformation matrix
    approachPose = transl(approachPosition) * targetOrientation;

    % Get the current joint positions of the robot
    qInitial = A.model.getpos();  % Current configuration of the robot

    % Solve inverse kinematics to find joint configuration for the approach position
    qApproach = A.model.ikcon(approachPose, qInitial);

    % Ensure all joint values are within their respective limits
    for i = 1:length(qApproach)
        qApproach(i) = min(max(qApproach(i), jointLimits(i,1)), jointLimits(i,2));
    end

    % Generate a trajectory from the initial position to the approach position
    steps = 100;  % Number of steps in the trajectory
    qMatrixApproach = jtraj(qInitial, qApproach, steps);  % Trajectory for the approach

    % Animate the robot moving to the approach position
    for i = 1:steps
        A.model.animate(qMatrixApproach(i,:));  % Animate each step
        drawnow();
        endEffectorPose = A.model.fkine(qMatrixApproach(i,:));
        currentPos = endEffectorPose.t
        if isgraphics(coin)
            delete(coin);
        end
        coin = PlaceCoinYellow(currentPos);
        drawnow();
        
    end
    if isgraphics(coin)
        delete(coin);
    end
    disp('Robot moved to the approach position.');
end

function MoveToApproachPosition(A, initialPosition, jointLimits)
    % Move the robot to a position directly above the brick (safe approach)
    % Returns qApproach (joint angles for the approach), qInitial (starting joint angles),
    % approachPose (target pose), and qMatrixApproach (trajectory) for later use
    % Define the position above the brick with the specified approach height
    approachPosition = [initialPosition(1), initialPosition(2), initialPosition(3)];

    % Set the end-effector orientation to face downward (roll = pi radians)
    targetOrientation = rpy2tr(pi, 0, 0);  % Roll = pi (180 degrees), end-effector points down

    % Combine the position and orientation into a homogeneous transformation matrix
    approachPose = transl(approachPosition) * targetOrientation;

    % Get the current joint positions of the robot
    qInitial = A.model.getpos();  % Current configuration of the robot

    % Solve inverse kinematics to find joint configuration for the approach position
    qApproach = A.model.ikcon(approachPose, qInitial);

    % Ensure all joint values are within their respective limits
    for i = 1:length(qApproach)
        qApproach(i) = min(max(qApproach(i), jointLimits(i,1)), jointLimits(i,2));
    end

    % Generate a trajectory from the initial position to the approach position
    steps = 100;  % Number of steps in the trajectory
    qMatrixApproach = jtraj(qInitial, qApproach, steps);  % Trajectory for the approach

    % Animate the robot moving to the approach position
    for i = 1:steps
        A.model.animate(qMatrixApproach(i,:));  % Animate each step
        drawnow();  % Update the figure
    end
    
    disp('Robot moved to the approach position.');
end

function MoveLinearTM5ToPoseWithCoin(robot, desiredPosition, desiredOrientation, coin2)
    % Ensure desiredOrientation is a homogeneous transformation matrix
    if size(desiredOrientation, 1) == 3 && size(desiredOrientation, 2) == 3
        desiredOrientation = [desiredOrientation, [0; 0; 0]; 0 0 0 1];
    elseif size(desiredOrientation, 1) ~= 4 || size(desiredOrientation, 2) ~= 4
        error('desiredOrientation must be a 3x3 rotation matrix or a 4x4 homogeneous transformation matrix.');
    end

    % Combine position and orientation into a homogeneous transformation
    desiredPose = transl(desiredPosition) * desiredOrientation;

    % Move the robot to the desired end-effector pose
    MoveToDesiredPosition(robot, desiredPose);

    % Nested function to perform the movement
    function MoveToDesiredPosition(robot, desiredPose)
        % Get the current joint positions
        qInitial = robot.model.getpos();

        % Solve inverse kinematics for the desired pose
        [qSolution, err] = robot.model.ikcon(desiredPose, qInitial);

        % Check for a valid solution
        if err > 1e-3
            warning('Could not find a valid joint configuration for the desired pose.');
            return;
        end

        % Generate a joint space trajectory from the initial to the solution joint positions
        steps = 100;  % Number of steps in the trajectory
        qMatrix = jtraj(qInitial, qSolution, steps);

        % Animate the robot movement along the trajectory
        for k = 1:steps
            robot.model.animate(qMatrix(k, :));
            drawnow();
            endEffectorPose2 = robot.model.fkine(qMatrix(k, :));
            currentPos = endEffectorPose2.t;
        if isgraphics(coin2)
            delete(coin2);
        end
        coin2 = PlaceCoinRed(currentPos);
        drawnow();
        end

        if isgraphics(coin2)
        delete(coin2);
        end

        disp('Moved to the desired end-effector pose.');
    end
end



function MoveLinearTM5ToPose(robot, desiredPosition, desiredOrientation)
    % Function to move the LinearTM5 robot to a desired end-effector pose without enforcing joint limits.
    %
    % Inputs:
    %   robot              - An instance of the LinearTM5 robot
    %   desiredPosition    - 1x3 vector [x, y, z] specifying the target position
    %   desiredOrientation - 3x3 rotation matrix or 4x4 homogeneous transformation matrix specifying the desired orientation
    %
    % Example usage:
    %   robot = LinearTM5();
    %   desiredPosition = [0.3, 0.0, 0.5];
    %   desiredOrientation = rpy2tr(pi, 0, 0);  % End-effector facing down
    %   MoveLinearTM5ToPose(robot, desiredPosition, desiredOrientation);

    % Ensure desiredOrientation is a homogeneous transformation matrix
    if size(desiredOrientation, 1) == 3 && size(desiredOrientation, 2) == 3
        desiredOrientation = [desiredOrientation, [0; 0; 0]; 0 0 0 1];
    elseif size(desiredOrientation, 1) ~= 4 || size(desiredOrientation, 2) ~= 4
        error('desiredOrientation must be a 3x3 rotation matrix or a 4x4 homogeneous transformation matrix.');
    end

    % Combine position and orientation into a homogeneous transformation
    desiredPose = transl(desiredPosition) * desiredOrientation;

    % Move the robot to the desired end-effector pose
    MoveToDesiredPosition(robot, desiredPose);

    % Nested function to perform the movement
    function MoveToDesiredPosition(robot, desiredPose)
        % Get the current joint positions
        qInitial = robot.model.getpos();

        % Solve inverse kinematics for the desired pose
        [qSolution, err] = robot.model.ikcon(desiredPose, qInitial);

        % Check for a valid solution
        if err > 1e-3
            warning('Could not find a valid joint configuration for the desired pose.');
            return;
        end

        % Generate a joint space trajectory from the initial to the solution joint positions
        steps = 100;  % Number of steps in the trajectory
        qMatrix = jtraj(qInitial, qSolution, steps);

        % Animate the robot movement along the trajectory
        for k = 1:steps
            robot.model.animate(qMatrix(k, :));
            drawnow();
        end

        disp('Moved to the desired end-effector pose.');
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


% Main Script: Simulate LinearTM5 Picking Up and Placing a Coin

% Initialize the LinearTM5 robot
robot = LinearTM5();
coinYellow = PlaceCoinYellow([-0.95, -0.2, -5]);
coinRed = PlaceCoinRed ([-0.85, -0.2, 0.005]);
Board = PlaceFlatObject([-0.5, 0.45, 0]);

% Define the pick-up and place-down positions and orientations
% We'll define positions on the ground at different locations

%first turn, UR3
pause(10);
MoveToApproachPosition(A, [-0.85, -0.2, 0.4], jointLimits);
MoveToApproachPosition(A, [-0.85, -0.2, 0.005], jointLimits);
MoveToApproachPositionWithCoin(A, [-0.85, -0.2, 0.4], jointLimits, coinRed);
MoveToApproachPositionWithCoin(A, [-0.61, 0.2, 0.4], jointLimits, coinRed);
MoveToApproachPositionWithCoin(A, [-0.61, 0.2, 0.008], jointLimits, coinRed);
coinRed = PlaceCoinRed([-0.61, 0.2, 0.008]);
MoveToApproachPosition(A, [-0.61, 0.2, 0.4], jointLimits);
%second turn, LinearTM5
placeDownPosition = [-0.4, 0.2, 0.2];  % Another position on the ground
placeDownPosition2 = [-0.4, 0.2, 0.15];
placeDownOrientation = rpy2tr(pi, 0, 0);  % End-effector facing down
disp('Moving to place-down position...');
MoveLinearTM5ToPoseWithCoin(robot, [-0.4, 0.2, 0.2], placeDownOrientation, coinYellow);
MoveLinearTM5ToPoseWithCoin(robot, [-0.4, 0.2, 0.15], placeDownOrientation, coinYellow);
coinYellow = PlaceCoinYellow([-0.4, 0.2, 0.008]);
MoveLinearTM5ToPose(robot, [-0.4, 0.2, 0.3], placeDownOrientation);
MoveLinearTM5ToPose(robot, [-0.4, 0.2, 0.2], placeDownOrientation);

%third turn, UR3
%MoveToApproachPosition(A, [-0.95, -0.2, 0.4], jointLimits);
%MoveToApproachPosition(A, [-0.95, -0.2, 0.005], jointLimits);
%MoveToApproachPositionWithCoin(A, [-0.95, -0.2, 0.4], jointLimits, );


