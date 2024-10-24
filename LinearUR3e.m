classdef LinearUR3e < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR3e';
    end
    
    methods


%% Constructor
function self = LinearUR3e(baseTr,useTool,toolFilename)
    if nargin < 3
        if nargin == 2
            error('If you set useTool you must pass in the toolFilename as well');
        elseif nargin == 0 % Nothing passed
            baseTr = transl(0,0,0.0);  % Move the robot base up slightly
        end             
    else % All passed in 
        self.useTool = useTool;
        toolTrData = load([toolFilename,'.mat']);
        self.toolTr = toolTrData.tool;
        self.toolFilename = [toolFilename,'.ply'];
    end
    
    self.CreateModel();
    
    % Rotate and translate the base to set the robot upright
    self.model.base = self.model.base.T * baseTr * trotx(pi/2);  % Rotate 90 degrees around X-axis
    
    % Plot the robot and adjust axis limits
    self.PlotAndColourRobot();
    axis([-1 1 -1 1 0 1]);  % Set custom axis limits for better visualization
    
    drawnow;
end
%% CreateModel
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link 
            link(2) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', -pi/2);
            link(4) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            link(1).qlim = [-0.7 -0.02];
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-80 80]*pi/180;
            link(4).qlim = [-80 80]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;  

             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end