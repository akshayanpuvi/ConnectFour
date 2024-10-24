% Test script to visualize each .ply file for LinearTM5 robot separately

% List of .ply files
plyFiles = {'TM5Link0.ply', 'TM5Link1.ply', 'TM5Link2.ply', ...
            'TM5Link3.ply', 'TM5Link4.ply', 'TM5Link5.ply', ...
            'TM5Link6.ply', 'TM5Link7.ply'};

% Loop through each .ply file and visualize it in a separate figure
for i = 1:length(plyFiles)
    % Create a new figure for each file
    figure;
    
    % Load the .ply file
    [f, v, data] = plyread(plyFiles{i}, 'tri');
    
    % Plot the 3D mesh
    trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    
    % Set title to the current .ply file name
    title(['Preview of ', plyFiles{i}]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Set axis properties and view
    axis equal;
    view(3);
    camlight;
    lighting gouraud;
end
