classdef LightCurtainSafetyEnvironment < handle
    properties
        safetyZone % Safety zone boundary
        objectsToMonitor % List of objects that need monitoring
    end
    
    methods
        function self = LightCurtainSafetyEnvironment()
            % Define the safety zone as a rectangular region in the x-y plane
            self.safetyZone = [-1.5, 1.5; -1.5, 1.5; 0, 3.5]; % [xmin xmax; ymin ymax; zmin zmax]
            self.objectsToMonitor = {};
        end
        
        function addObjectToMonitor(self, objectHandle)
            % Add an object handle to the list of monitored objects
            self.objectsToMonitor{end+1} = objectHandle;
        end
        
        function checkSafety(self)
            % Check if any monitored object has entered the safety zone
            for i = 1:length(self.objectsToMonitor)
                objectHandle = self.objectsToMonitor{i};
                vertices = get(objectHandle, 'Vertices');
                
                % Check if any vertex is within the safety zone
                if any(self.isInsideSafetyZone(vertices))
                    disp('Warning: Object has entered the light curtain safety zone!');
                    % Add code here to stop the robots or trigger a safety response
                    self.triggerSafetyAction();
                    break;
                end
            end
        end
        
        function isInside = isInsideSafetyZone(self, vertices)
            % Check if vertices are inside the safety zone
            xInside = vertices(:,1) >= self.safetyZone(1,1) & vertices(:,1) <= self.safetyZone(1,2);
            yInside = vertices(:,2) >= self.safetyZone(2,1) & vertices(:,2) <= self.safetyZone(2,2);
            zInside = vertices(:,3) >= self.safetyZone(3,1) & vertices(:,3) <= self.safetyZone(3,2);
            isInside = xInside & yInside & zInside;
        end
        
        function triggerSafetyAction(self)
            % Trigger a safety action when an object enters the safety zone
            disp('Safety Action: Robots stopped!');
            % Insert robot stop command here
            % For example: r1.Stop(); r2.Stop();
        end
    end
end
