
function robotMasterclassGUI
    global handles;
    handles.isStopped = false;
    handles.isPlaying = false; % Flag to track if animation is playing

    % Set up the main figure for Robot Masterclass GUI
    screenSize = get(0, 'ScreenSize');
    figWidth = 800;
    figHeight = 500;

    figX = (screenSize(3) - figWidth) / 2;
    figY = (screenSize(4) - figHeight) / 2;

    hFigure = figure('Position', [figX, figY, figWidth, figHeight], ...
        'Name', 'Robot Masterclass', ...
        'NumberTitle', 'off', ...
        'Color', [0.9, 0.9, 0.9], ...
        'Resize', 'off', ...
        'CloseRequestFcn', @(src, event) closeGUI(), ... % Override close action
        'KeyPressFcn', @keyPressHandler); % Add KeyPressFcn to detect spacebar presses

    % Panel for controls on the left
    controlPanel = uipanel('Parent', hFigure, 'Title', 'Controls', ...
        'Position', [0, 0, 0.4, 1], 'BackgroundColor', [0.9, 0.9, 0.9]);

    % Panel for robot simulation on the right
    simPanel = uipanel('Parent', hFigure, 'Title', 'Robot Simulation', ...
        'Position', [0.4, 0, 0.6, 1], 'BackgroundColor', [1, 1, 1]);

    % Axes for robot animation
    hAxes = axes('Parent', simPanel, 'Position', [0.1, 0.1, 0.8, 0.8]);

    % E-Stop button in control panel
    handles.eStopButton = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'E-Stop', ...
        'Position', [100, 400, 100, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', 'red', 'ForegroundColor', [1, 1, 1], ...
        'Callback', @toggleEStop);

    % Start button to start animation
    handles.startButton = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Start', ...
        'Position', [100, 340, 100, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', 'green', 'ForegroundColor', [1, 1, 1], ...
        'Callback', @startAnimation);

    % Return to Menu button
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Return to Menu', ...
        'Position', [100, 280, 100, 50], 'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1], ...
        'Callback', @(src, event) returnToMenu());

    % Joint Control sliders
    for i = 1:6
        uicontrol('Parent', controlPanel, 'Style', 'text', 'String', ['Joint ', num2str(i)], ...
            'Position', [20, 230 - (i-1)*30, 80, 20], 'FontSize', 10, ...
            'BackgroundColor', [0.9, 0.9, 0.9]);

        uicontrol('Parent', controlPanel, 'Style', 'slider', ...
            'Position', [120, 230 - (i-1)*30, 150, 20], ...
            'Min', -180, 'Max', 180, ...
            'Callback', @(src, event) move_joint(i, src.Value));
    end

    % Cartesian Control Inputs
    uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'X:', ...
        'Position', [20, 60, 50, 20], 'BackgroundColor', [0.9, 0.9, 0.9]);
    x_input = uicontrol('Parent', controlPanel, 'Style', 'edit', ...
        'Position', [50, 60, 50, 20]);

    uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'Y:', ...
        'Position', [120, 60, 50, 20], 'BackgroundColor', [0.9, 0.9, 0.9]);
    y_input = uicontrol('Parent', controlPanel, 'Style', 'edit', ...
        'Position', [150, 60, 50, 20]);

    uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'Z:', ...
        'Position', [220, 60, 50, 20], 'BackgroundColor', [0.9, 0.9, 0.9]);
    z_input = uicontrol('Parent', controlPanel, 'Style', 'edit', ...
        'Position', [250, 60, 50, 20]);

    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Move', ...
        'Position', [100, 20, 80, 30], ...
        'Callback', @(src, event) move_cartesian(str2double(get(x_input, 'String')), ...
                                                 str2double(get(y_input, 'String')), ...
                                                 str2double(get(z_input, 'String'))));

    % Toggle E-Stop (Pause/Resume) function
    function toggleEStop(~, ~)
        if handles.isPlaying
            handles.isStopped = ~handles.isStopped;
            if handles.isStopped
                set(handles.eStopButton, 'String', 'Resume');
                disp('Animation paused.');
            else
                set(handles.eStopButton, 'String', 'E-Stop');
                disp('Animation resumed.');
            end
        end
    end

    % Start Animation function
    function startAnimation(~, ~)
        if ~handles.isPlaying
            handles.isPlaying = true;
            disp('Starting animation...');
            animateRobot(); % Call the function to animate the robot
        else
            disp('Animation is already running.');
        end
    end

    % Animate Robot function (simulating robot movements)
    function animateRobot()
        while handles.isPlaying && ~handles.isStopped
            % Example animation code (replace with actual robot animation code)
            % For now, simulate a rotating joint or simple animation for demonstration
            cla(hAxes); % Clear axes
            plot(hAxes, rand(1, 10), rand(1, 10), 'r-'); % Example animation
            pause(0.5); % Pause for animation effect
        end
        disp('Animation stopped.');
    end

    % Return to Menu function
    function returnToMenu()
        handles.isPlaying = false; % Stop animation if running
        close(hFigure); % Close the current GUI
        StartUp(); % Reopen the Start-Up menu
    end

    % Close GUI safely
    function closeGUI()
        handles.isPlaying = false; % Stop any animation before closing
        delete(hFigure); % Close the GUI
    end

    % Key press handler for spacebar E-Stop
    function keyPressHandler(~, event)
        if strcmp(event.Key, 'space')
            toggleEStop(); % Call E-Stop toggle when spacebar is pressed
        end
    end

    % Placeholder function to move joint (can be integrated with actual robot)
    function move_joint(joint_idx, value)
        if handles.isStopped
            disp('System is stopped. Cannot move joints.');
            return;
        end
        % Update joint positions here
    end

    % Placeholder function to move in Cartesian space (can be integrated with actual robot)
    function move_cartesian(x, y, z)
        if handles.isStopped
            disp('System is stopped. Cannot move in Cartesian space.');
            return;
        end
        % Update Cartesian positions here
    end
end
