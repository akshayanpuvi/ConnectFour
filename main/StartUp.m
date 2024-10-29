function StartUp

    handles.isStopped = false; % Declare the global variable here

    % Create a new figure window for the GUI
    screenSize = get(0, 'ScreenSize');
    figWidth = 400;
    figHeight = 500;

    figX = (screenSize(3) - figWidth) / 2;
    figY = (screenSize(4) - figHeight) / 2;

    hFigure = figure('Position', [figX, figY, figWidth, figHeight], ...
        'Name', 'Connect Four Start-Up', ...
        'NumberTitle', 'off', ...
        'Color', [0.8, 0.9, 1], ...
        'Resize', 'off');

    % Add a title to the GUI
    uicontrol('Style', 'text', 'String', 'Welcome to Connect Four!', ...
        'Position', [70, 450, 300, 40], 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.8, 0.9, 1], 'HorizontalAlignment', 'center');

    % Add the "PLAY NOW" button
    uicontrol('Style', 'pushbutton', 'String', 'PLAY NOW', ...
        'Position', [170, 400, 100, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1], ...
        'Callback', @playNowCallback);

    % Add the "estop" button
    uicontrol('Style', 'pushbutton', 'String', 'E-Stop', ...
        'Position', [192, 5, 60, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', 'red', 'ForegroundColor', [1, 1, 1], ...
        'Callback', @estopCallback);
    %Add UR3e togglebutton
    uicontrol('Style', 'togglebutton', 'String', 'UR3e', ...
        'Position', [150, 340, 60, 50], 'FontSize', 12, 'FontWeight', 'normal', ...
        'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1]);

%   Add UR3e togglebutton
    uicontrol('Style', 'togglebutton', 'String', 'TM5', ...
        'Position', [230, 340, 60, 50], 'FontSize', 12, 'FontWeight', 'normal', ...
        'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1]);

    % Joint Control
    for i = 1:6
        uicontrol('Style', 'text', 'String', ['Joint ', num2str(i)], ...
            'Position', [20, 300 - (i-1)*30, 100, 20], 'FontSize', 12, ...
            'BackgroundColor', [0.8, 0.9, 1]);

        % Create slider for each joint
        uicontrol('Style', 'slider', 'Position', [120, 300 - (i-1)*30, 200, 20], ...
            'Min', -180, 'Max', 180, ...
            'Callback', @(src, event) move_joint(i, src.Value));
    end

    % Cartesian Control Inputs
    uicontrol('Style', 'text', 'Position', [45, 100, 50, 20], ...
        'BackgroundColor', [0.8, 0.9, 1], 'String', 'X:');
    x_input = uicontrol('Style', 'edit', 'Position', [80, 103, 50, 20]);
    uicontrol('Style', 'text', 'Position', [135, 100, 50, 20], ...
        'BackgroundColor', [0.8, 0.9, 1], 'String', 'Y:');
    y_input = uicontrol('Style', 'edit', 'Position', [170, 103, 50, 20]);
    uicontrol('Style', 'text', 'Position', [225, 100, 50, 20], ...
        'BackgroundColor', [0.8, 0.9, 1], 'String', 'Z:');
    z_input = uicontrol('Style', 'edit', 'Position', [260, 103, 50, 20]);

    uicontrol('Style', 'pushbutton', 'Position', [335, 103, 50, 20], ...
        'String', 'Move', ...
        'Callback', @(src, event) move_cartesian(str2double(get(x_input, 'String')), ...
                                                 str2double(get(y_input, 'String')), ...
                                                 str2double(get(z_input, 'String'))));

    % Callback function for the "E-Stop"
    function estopCallback(~, ~)
        handles.isStopped = true; % Declare the global variable here
        disp('E-Stop activated. System halted.');

        % Disable all controls
        set(findall(hFigure, 'Enable', 'on'), 'Enable', 'off');

        % Pause for 5 seconds
        pause(5);

        % Resume operation
        handles.isStopped = false;
        set(findall(hFigure, 'Enable', 'off'), 'Enable', 'on'); % Enable all controls
        disp('System resumed.');

    end

    % Move joint function
    function move_joint(joint_idx, value)
        if (handles.isStopped == true)
            disp('System is stopped. Cannot move joints.');
            return;
        end

        % Example for moving both robots (replace with your robot objects)
        % Assuming 'robot1' and 'robot2' are your robot instances
        q1 = robot1.q; % Get current joint angles for robot1
        q2 = robot2.q; % Get current joint angles for robot2

        % Update joint position
        q1(joint_idx) = value; % Update robot1 joint
        q2(joint_idx) = value; % Update robot2 joint

        % Animate both robots
        robot1.animate(q1);
        robot2.animate(q2);
    end

    % Move Cartesian function
    function move_cartesian(x, y, z)
        if (handles.isStopped == true)
            disp('System is stopped. Cannot move in Cartesian space.');
            return;
        end

        % Define your transformation and IK for both robots
        T = transl(x, y, z);  % Transformation matrix
        q1 = robot1.ikine(T); % Inverse kinematics for robot1
        q2 = robot2.ikine(T); % Inverse kinematics for robot2

        % Animate both robots
        robot1.animate(q1);
        robot2.animate(q2);
    end

    % Callback function for the "PLAY NOW" button
    function playNowCallback(~, ~)
        choice = questdlg('Choose Your Game Mode:', 'Select Mode', ...
            'Robot Masterclass', 'Single-Player', 'Cancel', 'Cancel');

        switch choice
            case 'Robot Masterclass'
                disp('Starting Robot Masterclass (Two Robots Playing)...');
                close(hFigure);
                % Launch Robot vs Robot GUI here
                % RobotMasterclass;  % Uncomment this line when ready

            case 'Single-Player'
                disp('Starting Single-Player (Human vs Robot)...');
                close(hFigure);
                % Launch Single-Player GUI here
                % SinglePlayer;  % Uncomment this line when ready

            case 'Cancel'
                disp('Action canceled.');
        end
    end
end