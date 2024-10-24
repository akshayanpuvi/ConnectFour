classdef Pulse90 < RobotBaseClass
    %% Pulse90 Robot Model with Linear Rail and Tool Support
    %
    % WARNING: This model has been created based on available information.
    % No guarantee is made about the accuracy or correctness of the DH parameters
    % or the accompanying ply files. Do not assume that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'Rozum';  % The prefix for your .ply files
    end

    methods
        %% Constructor
        function self = Pulse90(baseTr, useTool, toolFilename)
            % Initialize the Pulse90 robot model with optional base translation and tool
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0  % Nothing passed
                    baseTr = transl(0, 0, 0);  % Default base translation
                end
            else  % All arguments passed
                self.useTool = useTool;
                toolTrData = load([toolFilename, '.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename, '.ply'];
            end

            self.CreateModel();  % Create the robot model

            % Adjust the robot base orientation
            self.model.base = self.model.base.T * baseTr * trotx(pi/2);  % Rotate 90 degrees around X-axis

            % Plot and visualize the robot
            self.PlotAndColourRobot();
            axis([-1 1 -1 1 0 1]);  % Adjust axis limits for better visualization
            drawnow;
        end

        %% Create the DH model of the robot
        function CreateModel(self)
            % Define the DH parameters for Pulse90 robot
            link(1) = Link([pi     0       0       pi/2    1]);  % PRISMATIC Link 
            link(2) = Link('d', 0.2325, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(3) = Link('d', 0, 'a', 0.450, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d', 0, 'a', 0.370, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d', 0.1205, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(6) = Link('d', 0.0975, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(7) = Link('d', 0.171, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);

            % Define joint limits for each link
            link(1).qlim = [-0.7 -0.02];
            link(2).qlim = [-90 90] * pi/180;
            link(3).qlim = [-80 80] * pi/180;
            link(4).qlim = [-80 80] * pi/180;
            link(5).qlim = [-360 360] * pi/180;
            link(6).qlim = [-360 360] * pi/180;
            link(7).qlim = [-360 360] * pi/180;

            % Create the robot model using SerialLink
            self.model = SerialLink(link, 'name', 'Pulse 90');
        end

        %% Load the .ply files, attach to the robot, and plot
        function PlotAndColourRobot(self)
            % Iterate through each link of the robot
            for linkIndex = 0:self.model.n-1
                plyFilePath = [self.plyFileNameStem, num2str(linkIndex), '.ply'];  % Construct the file path
                [faceData, vertexData] = plyread(plyFilePath, 'tri');  % Load the .ply file

                % Render the .ply file mesh as a patch object
                patch('Faces', faceData, 'Vertices', vertexData, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none');
                hold on;
            end

            % Plot the robot's joints and configuration
            self.model.plot(zeros(1, self.model.n), 'workspace', [-1 1 -1 1 0 1], 'view', [30 30]);
            camlight;  % Add lighting
            hold off;
        end
    end
end
