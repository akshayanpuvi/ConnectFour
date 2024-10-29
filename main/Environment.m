function Environment
            clf;
            clc;
            hold on;
            axis equal;
            axis on;
            grid on;

            axis([-5.3 5.3 -4.5 4 -0.1 4])     % Setting the workspace axis [xmin xmax ymin ymax zmin zmax]

            view(3);    % Helps with visualising the figure

            wall1 = PlaceObject('tile.ply', [0.274, 0.95, -2.8]);    
            vertwall1 = [get(wall1,'Vertices'), ones(size(get(wall1,'Vertices'),1),1)]*trotx(-pi/2);     
            vertwall1(:,1) = vertwall1(:,1) * 7.2;    
            vertwall1(:,2) = vertwall1(:,2) * 1;  
            vertwall1(:,3) = vertwall1(:,3) * 2;
            set(wall1,'Vertices',vertwall1(:,1:3))

            wall2 = PlaceObject('tile.ply', [0.95, -0.1, -4.3]);    
            vertwall2 = [get(wall2,'Vertices'), ones(size(get(wall2,'Vertices'),1),1)]*troty(pi/2);     
            vertwall2(:,1) = vertwall2(:,1) * 1;    
            vertwall2(:,2) = vertwall2(:,2) * 4.25;  
            vertwall2(:,3) = vertwall2(:,3) * 2.0;
            set(wall2,'Vertices',vertwall2(:,1:3))

            floor = PlaceObject('tile.ply', [-0.27, -0.1, 1.05]);    
            vertfloor = [get(floor,'Vertices'), ones(size(get(floor,'Vertices'),1),1)]*trotx(pi)*trotz(pi);     
            vertfloor(:,1) = vertfloor(:,1) * 7.2;    
            vertfloor(:,2) = vertfloor(:,2) * 4.2;  
            vertfloor(:,3) = vertfloor(:,3) * 1;
            set(floor,'Vertices',vertfloor(:,1:3))

            home = PlaceObject('ImageToStl.com_house.ply', [0, 0, -100]);    
            verthome = [get(home,'Vertices'), ones(size(get(home,'Vertices'),1),1)];     
            verthome(:,1) = verthome(:,1) * 0.022;    
            verthome(:,2) = verthome(:,2) * 0.022;  
            verthome(:,3) = verthome(:,3) * 0.022;
            set(home,'Vertices',verthome(:,1:3))

            table = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [-1.0, -0.5, 0]);    % Line 1
            verttable = [get(table,'Vertices'), ones(size(get(table,'Vertices'),1),1)];     % Line 2
            verttable(:,2) = verttable(:,2) * 2;    % Line 3
            verttable(:,1) = verttable(:,1) * 2;  % Line 4
            set(table,'Vertices',verttable(:,1:3))  % Line 5

            % baseTr1 = transl([-2.35, -1.001, 0.5])* trotz(pi/2);    % Translation in x, y, z and rotation about the z axis clockwise sets the where the base of the robot is located
            % r1 = UR3(baseTr1);     % Modelling the UR3e on Linear rails with the base transform
            % baseTr2 = transl([-1.4, -0.7, 0.5]) * trotz(pi);    % Translation in x, y, z and rotation about the z axis clockwise sets the where the base of the robot is located
            % r2 = LinearTM5(baseTr2);     % Modelling the UR3e on Linear rails with the base transform

            flatc4 = PlaceObject('flatc4.ply', [20, -10.25, -40]);    
            vertflatc4 = [get(flatc4,'Vertices'), ones(size(get(flatc4,'Vertices'),1),1)]*trotx(pi/2)*trotz(pi/2);     
            vertflatc4(:,1) = vertflatc4(:,1) * 0.05;    
            vertflatc4(:,2) = vertflatc4(:,2) * 0.05;  
            vertflatc4(:,3) = vertflatc4(:,3) * 0.05;
            set(flatc4,'Vertices',vertflatc4(:,1:3))

            for x = -2700:-300:-3300
                for y = -1800:-300:-3600
                    countery = PlaceObject('ImageToStl.com_countery.ply', [x, y, 1450]);    
                    vertcountery = [get(countery,'Vertices'), ones(size(get(countery,'Vertices'),1),1)];     
                    vertcountery(:,1) = vertcountery(:,1) * 0.00035;    
                    vertcountery(:,2) = vertcountery(:,2) * 0.00035;  
                    vertcountery(:,3) = vertcountery(:,3) * 0.00035;
                    set(countery,'Vertices',vertcountery(:,1:3))
                end
            end

            for x = -7500:-300:-8100
                for y = -1800:-300:-3600
                    counterr = PlaceObject('ImageToStl.com_counter.ply', [x, y, 1450]);    
                    vertcounterr = [get(counterr,'Vertices'), ones(size(get(counterr,'Vertices'),1),1)];     
                    vertcounterr(:,1) = vertcounterr(:,1) * 0.00035;    
                    vertcounterr(:,2) = vertcounterr(:,2) * 0.00035;  
                    vertcounterr(:,3) = vertcounterr(:,3) * 0.00035;
                    set(counterr,'Vertices',vertcounterr(:,1:3))
                end
            end

            personstand = PlaceObject('personMaleCasual.ply', [-0.5, 0.7, 0]);    
            vertpersonstand = [get(personstand,'Vertices'), ones(size(get(personstand,'Vertices'),1),1)]*trotz(pi);     
            vertpersonstand(:,1) = vertpersonstand(:,1) * 1.5;    
            vertpersonstand(:,2) = vertpersonstand(:,2) * 1.5;  
            vertpersonstand(:,3) = vertpersonstand(:,3) * 1.5;
            set(personstand,'Vertices',vertpersonstand(:,1:3))

            %PlaceObject('emergencyStopWallMounted.ply', [-1, -2.45, 1]);

            estop = PlaceObject('emergencyStopWallMounted.ply', [1, -5.2, 1.5]);    
            vertestop = [get(estop,'Vertices'), ones(size(get(estop,'Vertices'),1),1)]*trotz(pi/2);     
            set(estop,'Vertices',vertestop(:,1:3))

            firex = PlaceObject('fireExtinguisher.ply', [-5.2, -2, 1.25]);    
            vertfirex = [get(firex,'Vertices'), ones(size(get(firex,'Vertices'),1),1)];     
            set(firex,'Vertices',vertfirex(:,1:3))

            slightsensor = PlaceObject('safetycurtain.ply', [-54, 4, 6.8]);    
            vertslightsensor = [get(slightsensor,'Vertices'), ones(size(get(slightsensor,'Vertices'),1),1)];     
            vertslightsensor(:,1) = vertslightsensor(:,1) * 0.075;    
            vertslightsensor(:,2) = vertslightsensor(:,2) * 0.075;  
            vertslightsensor(:,3) = vertslightsensor(:,3) * 0.075;
            set(slightsensor,'Vertices',vertslightsensor(:,1:3))

            slightsensor2 = PlaceObject('safetycurtain.ply', [0.3, -5, 6.8]);    
            vertslightsensor2 = [get(slightsensor2,'Vertices'), ones(size(get(slightsensor2,'Vertices'),1),1)]*trotz(pi);     
            vertslightsensor2(:,1) = vertslightsensor2(:,1) * 0.075;    
            vertslightsensor2(:,2) = vertslightsensor2(:,2) * 0.075;  
            vertslightsensor2(:,3) = vertslightsensor2(:,3) * 0.075;
            set(slightsensor2,'Vertices',vertslightsensor2(:,1:3))

            slightsensor3 = PlaceObject('safetycurtain.ply', [-54, -31, 6.8]);    
            vertslightsensor3 = [get(slightsensor3,'Vertices'), ones(size(get(slightsensor3,'Vertices'),1),1)];     
            vertslightsensor3(:,1) = vertslightsensor3(:,1) * 0.075;    
            vertslightsensor3(:,2) = vertslightsensor3(:,2) * 0.075;  
            vertslightsensor3(:,3) = vertslightsensor3(:,3) * 0.075;
            set(slightsensor3,'Vertices',vertslightsensor3(:,1:3))

            slightsensor4 = PlaceObject('safetycurtain.ply', [0.3, 30, 6.8]);    
            vertslightsensor4 = [get(slightsensor4,'Vertices'), ones(size(get(slightsensor4,'Vertices'),1),1)]*trotz(pi);     
            vertslightsensor4(:,1) = vertslightsensor4(:,1) * 0.075;    
            vertslightsensor4(:,2) = vertslightsensor4(:,2) * 0.075;  
            vertslightsensor4(:,3) = vertslightsensor4(:,3) * 0.075;
            set(slightsensor4,'Vertices',vertslightsensor4(:,1:3))

end