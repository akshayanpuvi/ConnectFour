classdef Connect4Environment < handle
    
    methods(Static)
        function Environment()
      
            clf;
            clc;
            hold on;
            axis equal;
            axis on;
            grid on;
            
            %axis([-5 5 -5 5 -5 10])     % Setting the workspace axis [xmin xmax ymin ymax zmin zmax]

            view(3);    % Helps with visualising the figure


            wall = PlaceObject('wall.ply', [0.75, -0.5, 0]);    
            vertwall = [get(wall,'Vertices'), ones(size(get(wall,'Vertices'),1),1)]*trotx(-pi/2);     
            vertwall(:,1) = vertwall(:,1) * 4;    
            vertwall(:,2) = vertwall(:,2) * 4;  
            vertwall(:,3) = vertwall(:,3) * 4;
            set(wall,'Vertices',vertwall(:,1:3))

            house = PlaceObject('tableBlue1x1x0.5m.ply', [0, 0, 0]);    
            verthouse = [get(house,'Vertices'), ones(size(get(house,'Vertices'),1),1)];     
            % verthouse(:,1) = verthouse(:,1);    
            % verthouse(:,2) = verthouse(:,2);  
            % verthouse(:,3) = verthouse(:,3);
            set(house,'Vertices',verthouse(:,1:3))

            pause();
            
            countery = PlaceObject('countery.ply', [0, 0, 0]);    
            vertcountery = [get(countery,'Vertices'), ones(size(get(countery,'Vertices'),1),1)]*trotx(pi/2);     
            vertcountery(:,1) = vertcountery(:,1) * 0.0025;    
            vertcountery(:,2) = vertcountery(:,2) * 0.0025;  
            vertcountery(:,3) = vertcountery(:,3) * 0.0025;
            set(countery,'Vertices',vertcountery(:,1:3))
            
            counterr = PlaceObject('counterr.ply', [1, 1, 1]);    
            vertcounterr = [get(counterr,'Vertices'), ones(size(get(counterr,'Vertices'),1),1)]*trotx(pi/2);     
            vertcounterr(:,1) = vertcounterr(:,1) * 0.0025;    
            vertcounterr(:,2) = vertcounterr(:,2) * 0.0025;  
            vertcounterr(:,3) = vertcounterr(:,3) * 0.0025;
            set(counterr,'Vertices',vertcounterr(:,1:3))

            pause();

            baseTr = transl([0, -0.6, 0.55]) * trotz(-pi/2);    % Translation in x, y, z and rotation about the z axis clockwise sets the where the base of the robot is located
            r = LinearUR3e(baseTr);     % Modelling the UR3e on Linear rails with the base transform

            table = PlaceObject('tableBlue1x1x0.5m.ply', [0, 0, 0]);    % Line 1
            verttable = [get(table,'Vertices'), ones(size(get(table,'Vertices'),1),1)];     % Line 2
            verttable(:,2) = verttable(:,2) * 2;    % Line 3
            verttable(:,1) = verttable(:,1) * 1.5;  % Line 4
            set(table,'Vertices',verttable(:,1:3))  % Line 5

            fence1 = PlaceObject('barrier1.5x0.2x1m.ply', [0, 1.6, 0]);
            vertf1 = [get(fence1,'Vertices'), ones(size(get(fence1,'Vertices'),1),1)];
            vertf1(:,1) = vertf1(:,1) * 2;
            set(fence1,'Vertices',vertf1(:,1:3))

            fence2 = PlaceObject('barrier1.5x0.2x1m.ply', [0, -1.6, 0]);
            vertf2 = [get(fence2,'Vertices'), ones(size(get(fence2,'Vertices'),1),1)];
            vertf2(:,1) = vertf2(:,1) * 2;
            set(fence2,'Vertices',vertf2(:,1:3))

            fence3 = PlaceObject('barrier1.5x0.2x1m.ply', [0, 1.6, 0]);
            vertf3 = [get(fence3,'Vertices'), ones(size(get(fence3,'Vertices'),1),1)] * trotz(pi/2);
            vertf3(:,2) = vertf3(:,2) * 2;
            set(fence3,'Vertices',vertf3(:,1:3))

            fence4 = PlaceObject('barrier1.5x0.2x1m.ply', [0, -1.6, 0]);
            vertf4 = [get(fence4,'Vertices'), ones(size(get(fence4,'Vertices'),1),1)] * trotz(pi/2);
            vertf4(:,2) = vertf4(:,2) * 2;
            set(fence4,'Vertices',vertf4(:,1:3))

            PlaceObject('emergencyStopWallMounted.ply', [-1, -2.45, 1]);



            for y = -0.15:-0.2:-0.55
                for x = 0.45:0.1:0.65
                    PlaceObject('HalfSizedRedGreenBrick.ply', [x, y, 0.55]);
                end
            end       
        end

        function Actuation()
        


        end

        function DataPlotting()
        


        end
    end

    methods
        function self = Connect4Environment

            self.Environment;
            self.Actuation;
            self.DataPlotting;

        end
    end
end