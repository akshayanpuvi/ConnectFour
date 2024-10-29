
function StartUp
    % Initial Start-Up GUI

    screenSize = get(0, 'ScreenSize');
    figWidth = 400;
    figHeight = 200;

    figX = (screenSize(3) - figWidth) / 2;
    figY = (screenSize(4) - figHeight) / 2;

    hFigure = figure('Position', [figX, figY, figWidth, figHeight], ...
        'Name', 'Connect Four Start-Up', ...
        'NumberTitle', 'off', ...
        'Color', [0.8, 0.9, 1], ...
        'Resize', 'off');

    % Title
    uicontrol('Parent', hFigure, 'Style', 'text', ...
        'String', 'Welcome to Connect Four!', ...
        'Position', [50, 140, 300, 40], ...
        'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.8, 0.9, 1], ...
        'HorizontalAlignment', 'center');

    % PLAY NOW button
    uicontrol('Parent', hFigure, 'Style', 'pushbutton', ...
        'String', 'PLAY NOW', ...
        'Position', [150, 70, 100, 50], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1], ...
        'Callback', @playNowCallback);

    % Callback function for the "PLAY NOW" button
    function playNowCallback(~, ~)
        % Prompt user for game mode selection
        choice = questdlg('Choose Your Mode:', 'Select Mode', ...
            'Robot Masterclass', 'Collision Detection', 'Cancel', 'Cancel');

        switch choice
            case 'Robot Masterclass'
                disp('Starting Robot Masterclass (Two Robots Playing)...');
                close(hFigure);
                % Open the advanced control GUI for Robot Masterclass mode
                robotMasterclassGUI();

            case 'Collision Detection'
                disp('Starting Collision Detection mode ...');
                close(hFigure);
                % Open the collision detection GUI
                collisionDetectionGUI();

            case 'Cancel'
                disp('Action canceled.');
        end
    end
end

