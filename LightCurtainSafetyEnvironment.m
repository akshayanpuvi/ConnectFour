classdef LightCurtainSafety < handle
    properties
        safetyZone % Safety zone boundary
        objectsToMonitor % List of objects that need monitoring
    end
    
    methods
        function self = LightCurtainSafety()
            % Define the safety zone as a rectangular region in the x-y plane
            self.safetyZone = [-1.5, 1.5; -1.5, 1.5; 0, 3.5]; % [xmin xmax; ymin ymax; zmin zmax]
            self.objectsToMonitor = {};
        end
        
        function addObjectToMonitor(self, objectHandle)
            % Add an object handle to the list of monitored objects
            self.objectsToMonitor{end+1} = objectHandle;
        end