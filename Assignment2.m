
%% Define the robot and joint limits
r1 = UR3;
jointLimits = r1.model.qlim;

% Function to move the robot to the pickup position
function [qApproach, qInitial, approachPose, qMatrixApproach] = MoveToPickupPosition(r1, pickupPosition, jointLimits)
    % Set the end-effector orientation to face downward
    targetOrientation = rpy2tr(-pi/2, 0, 0);  % End-effector points down
    
    % Approach height before picking up the coin
    approachHeight = 0.2;  % Safe height above the coin
    
    % Create approach pose
    approachPose = transl([pickupPosition(1), pickupPosition(2), approachHeight]) * targetOrientation;
    
    % Get initial joint configuration
    qInitial = r1.model.getpos();
    
    % Solve inverse kinematics for the approach position
    qApproach = r1.model.ikcon(approachPose, qInitial);
    
    % Ensure joint limits are respected
    for i = 1:length(qApproach)
        qApproach(i) = min(max(qApproach(i), jointLimits(i,1)), jointLimits(i,2));
    end
    
    % Generate trajectory
    steps = 1000;
    qMatrixApproach = jtraj(qInitial, qApproach, steps);
    
    % Animate the movement
    for i = 1:steps
        r1.model.animate(qMatrixApproach(i,:));
        drawnow();
    end
    disp('Moved to the pickup position.');
end

% Function to move the robot to the specified drop position
function MoveToDropPosition(r1, dropPosition, jointLimits, qApproach)
    targetOrientation = rpy2tr(pi, 0, 0);  % Maintain end-effector orientation
    
    % Create pose for hovering over the drop position
    hoverPose = transl(dropPosition) * targetOrientation;
    
    % Solve inverse kinematics for hover position
    qDrop = r1.model.ikcon(hoverPose, qApproach);
    
    % Ensure joint limits are respected
    for i = 1:length(qDrop)
        qDrop(i) = min(max(qDrop(i), jointLimits(i,1)), jointLimits(i,2));
    end
    
    % Generate trajectory
    steps = 1000;
    qMatrixDrop = jtraj(qApproach, qDrop, steps);
    
    % Animate the movement
    for i = 1:steps
        r1.model.animate(qMatrixDrop(i,:));
        drawnow();
    end
    disp('Moved to the specified drop position.');
end

% Main Script: Define pickup point and move to the desired drop point
% Step 1: Move to the pickup point
pickupPosition = [0.4, 0.4, 0.3];  % Define where the Connect 4 coin is placed
[qApproach, qInitial, approachPose, qMatrixApproach] = MoveToPickupPosition(r1, pickupPosition, jointLimits);

% Step 2: Define a specific desired drop point
desiredDropPosition = [-0.5, -0.5, 0.5];  % Define the desired x, y, z coordinates for the drop

% Step 3: Move to the specified drop position
MoveToDropPosition(r1, desiredDropPosition, jointLimits, qApproach);




