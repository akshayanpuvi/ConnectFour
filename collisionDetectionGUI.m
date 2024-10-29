function collisionDetectionGUI
    global handles;
    handles.isStopped = false;
    handles.isPlaying = false; % Flag to track if animation is playing

    % Set up the main figure for Collision Detection GUI
    screenSize = get(0, 'ScreenSize');
    figWidth = 800;
    figHeight = 500;

    figX = (screenSize(3) - figWidth) / 2;
    figY = (screenSize(4) - figHeight) / 2;

    hFigure = figure('Position', [figX, figY, figWidth, figHeight], ...
        'Name', 'Collision Detection Mode', ...
        'NumberTitle', 'off', ...
        'Color', [0.9, 0.9, 0.9], ...
        'Resize', 'off', ...
        'CloseRequestFcn', @(src, event) closeGUI(), ...
        'KeyPressFcn', @keyPressHandler); % Add KeyPressFcn to detect spacebar presses

    % Panel for controls on the left
    controlPanel = uipanel('Parent', hFigure, 'Title', 'Controls', ...
        'Position', [0, 0, 0.4, 1], 'BackgroundColor', [0.9, 0.9, 0.9]);

    % Panel for collision detection simulation on the right
    simPanel = uipanel('Parent', hFigure, 'Title', 'Collision Detection Simulation', ...
        'Position', [0.4, 0, 0.6, 1], 'BackgroundColor', [1, 1, 1]);

    % Axes for collision detection animation
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
            disp('Starting collision detection animation...');
            animateCollisionDetection(); % Call the function to animate the demo
        else
            disp('Animation is already running.');
        end
    end

    % Animate Collision Detection function (simulating collision detection)
    function animateCollisionDetection()
        while handles.isPlaying && ~handles.isStopped
            % Example animation code for collision detection (replace with actual animation)
            cla(hAxes); % Clear axes
            plot(hAxes, rand(1, 10), rand(1, 10), 'b-'); % Example animation
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
end
