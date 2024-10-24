% Visualize individual .ply files
plyFileStem = 'C:\Users\aksha\OneDrive\Documents\GitHub\ConnectFour\Pulse90\Rozum';  % Full path to the .ply files

figure;  % Create a new figure for visualization
for linkIndex = 0:7  % Loop through all the .ply files Rozum0.ply to Rozum7.ply
    
    % Construct the full file path for the current .ply file
    plyFilePath = [plyFileStem, num2str(linkIndex), '.ply'];
    
    % Load the .ply file
    [faceData, vertexData, plyData] = plyread(plyFilePath, 'tri');
    
    % Create a new subplot for each .ply file
    subplot(2, 4, linkIndex+1);  % Create a 2x4 grid of subplots (modify grid if necessary)
    
    % Plot the 3D mesh
    trisurf(faceData, vertexData(:, 1), vertexData(:, 2), vertexData(:, 3), 'EdgeColor', 'none');
    title(['Rozum', num2str(linkIndex), '.ply']);  % Title for each subplot
    
    % Set axis properties
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    camlight;  % Add lighting for better visualization
    lighting gouraud;
end

% Adjust figure properties
sgtitle('Visualization of Rozum .ply Files');
