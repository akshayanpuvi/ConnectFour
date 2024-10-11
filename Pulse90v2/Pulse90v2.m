classdef Pulse90v2 < RobotBaseClass

    properties(Access = public)
        plyFileNameStem = 'Pulse90v2';
    end

    methods
        %% Define robot Function
        function self = Pulse90v2(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();

        end

        function PlotAndColourRobot(self)
            figure;
            self.model.plot(zeros(1, self.model.n), 'workspace', [-1 1 -1 1 -1 1], 'tilesize', 0.1, 'floorlevel', -0.5);
        end


        %% Create the robot model
        function CreateModel(self)
            link(1) = Link('d', 0.2325, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d', 0, 'a', -0.450, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(3) = Link('d', 0, 'a', -0.370, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d', 0.1205, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d', 0.1711, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-360,360]), 'offset', 0);
            link(6) = Link('d', 0.1226, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360,360]), 'offset', 0);

            % Joint limits
            link(1).qlim = [-360 360]*pi/180; % Axis 1 (base) (-360° to 360°) [180°/s]
            link(2).qlim = [-360 360]*pi/180; % Axis 2 (-360° to 360°) [180°/s]
            link(3).qlim = [-160 160]*pi/180; % Axis 3 (-160° to 160°) [180°/s]
            link(4).qlim = [-360 360]*pi/180; % Axis 4 (-360° to 360°) [230°/s]
            link(5).qlim = [-360 360]*pi/180; % Axis 5 (-360° to 360°) [230°/s]
            link(6).qlim = [-360 360]*pi/180; % Axis 6 (-360° to 360°) [180°/s]

            % link(2).offset = -pi/2;
            % link(4).offset = -pi/2;

            self.model = SerialLink(link, 'name', self.name);
        end

    end
end
