function StartUp
% Create a new figure window for the GUI
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

% Add a title to the GUI
uicontrol('Style', 'text', 'String', 'Welcome to Connect Four!', ...
    'Position', [50, 130, 300, 40], 'FontSize', 14, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.8, 0.9, 1], 'HorizontalAlignment', 'center');

% Add the "PLAY NOW" button
uicontrol('Style', 'pushbutton', 'String', 'PLAY NOW', ...
    'Position', [150, 50, 100, 50], 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.2, 0.6, 0.8], 'ForegroundColor', [1, 1, 1], ...
    'Callback', @playNowCallback);

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