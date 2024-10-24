classdef Pulse90 < handle
    properties
        model;
        plyFileNameStem;
    end
    
    methods
        % Constructor to initialize the robot
        function self = Pulse90()
            % Set the full path to your .ply files
            self.plyFileNameStem = 'C:\Users\aksha\OneDrive\Documents\GitHub\ConnectFour\Pulse90\Rozum';  
            self.CreateModel();  % Call to create the robot model
            self.PlotRobot();    % Visualize the robot with .ply files
        end
        
        % Create the DH model of the robot
        function CreateModel(self)
            % Define the DH parameters for Pulse90 robot
            L(1) = Link('d', 0.2325, 'a', 0, 'alpha', pi/2);  % Link 1 (Base)
            L(2) = Link('d', 0, 'a', 0.450, 'alpha', 0);      % Link 2
            L(3) = Link('d', 0, 'a', 0.370, 'alpha', 0);      % Link 3
            L(4) = Link('d', 0.1205, 'a', 0, 'alpha', pi/2);  % Link 4
            L(5) = Link('d', 0.0975, 'a', 0, 'alpha', -pi/2); % Link 5
            L(6) = Link('d', 0.171, 'a', 0, 'alpha', 0);      % Link 6 (End-effector)
            
            % Create the robot model using SerialLink
            self.model = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)], 'name', 'Pulse 90');
        end
        
        % Load the .ply files, attach to robot, and plot
        function PlotRobot(self)
            % Iterate through each link of the robot
            for linkIndex = 0:self.model.n-1
                plyFilePath = [self.plyFileNameStem, num2str(linkIndex), '.ply'];  % Construct the file path
                [faceData, vertexData] = plyread(plyFilePath, 'tri');  % Load the .ply file
                
                % Render the .ply file mesh as a patch object
                patch('Faces', faceData, 'Vertices', vertexData, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none');
                hold on;
            end
            
            % Plot the robot's joints and configuration
            self.model.plot(zeros(1, 6), 'workspace', [-1 1 -1 1 0 1], 'view', [30 30]);
            camlight;  % Add lighting
            hold off;
        end
    end
end
