classdef LinearTM5 < RobotBaseClass

    %% LinearTM5 Robot model for educational purposes
    properties(Access = public)   
        plyFileNameStem = 'TM5';  % File name stem for TM5's 3D model (.ply)
    end

    methods(Access = public)

        %% Constructor
        % Constructs the TM5 robot and applies base transformations or tool settings if provided.
        function self = LinearTM5(baseTr,useTool,toolFilename)
            self.CreateModel();  % Create the TM5 model
            
            if nargin == 1
                baseTr = baseTr * trotx(pi/2);  % Rotate base by 90 degrees around X-axis
            elseif nargin == 0
                baseTr = transl(0,0,0) * trotx(pi/2) * troty(-pi/2);  % Default base translation and rotation
            elseif nargin == 3
                self.useTool = useTool;
                toolTrData = load([toolFilename, '.mat']);  % Load tool transformation
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename, '.ply'];  % Tool model file
            end
            
            self.model.base = self.model.base.T * baseTr;  % Apply base transformation
            self.model.tool = self.toolTr;  % Set tool
            self.PlotAndColourRobot();  % Plot the robot
        end
        
        %% Model Creation
        % Defines the DH parameters for the TM5 robot and creates the model.
        function CreateModel(self)
            link(1) = Link([pi 0 0 pi/2 1]);  % Custom primatic joint same as our LinearUR5
            link(2) = Link('d',0.1452,'a',0,'alpha',-pi/2,'qlim',deg2rad([-270, 270]), 'offset',0);
            link(3) = Link('d',0,'a',.329,'alpha',0,'qlim', deg2rad([-180, 180]), 'offset',0);
            link(4) = Link('d',0,'a', .3115,'alpha',0,'qlim', deg2rad([-180, 180]), 'offset', 0);
            link(5) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]),'offset', 0);
            link(6) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]), 'offset',0);
            link(7) = Link('d',0.1144,'a',0,'alpha',0,'qlim',deg2rad([-270, 270]), 'offset', 0);
            
            link(1).qlim = [-0.75 -0.025];  % Joint Limits
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-80 80]*pi/180;
            link(4).qlim = [-80 80]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;  

             
            self.model = SerialLink(link, 'name', self.name);  % Create SerialLink model
        end 
    end
end
