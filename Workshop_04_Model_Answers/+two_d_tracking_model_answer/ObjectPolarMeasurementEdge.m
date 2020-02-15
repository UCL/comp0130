classdef ObjectPolarMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        % The x,y and theta of the sensor
        sensorPose;
    end
    
    methods(Access = public)
    
        function this = ObjectPolarMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
            this.sensorPose = zeros(3, 1);
        end
        
        function setSensorPose(this, sensorPose)
            this.sensorPose = sensorPose;
        end
        
        function computeError(this)
            dXY = this.edgeVertices{1}.x([1 3]) - this.sensorPose(1:2);
            this.errorZ(1) = norm(dXY) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dXY(2), dXY(1)) - this.sensorPose(3) - this.z(2));
        end
        
        function linearizeOplus(this)
            dXY = this.edgeVertices{1}.x([1 3]) - this.sensorPose(1:2);
            r = norm(dXY);
            
            % Work out Jacobians
            this.J{1} = [dXY(1)/r 0 dXY(2)/r 0;
                -dXY(2)/r^2 0 dXY(1)/r 0];
        end        
    end
end