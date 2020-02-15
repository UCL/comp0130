classdef ObjectMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = ObjectMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            this.errorZ = this.edgeVertices{1}.x([1 3]) - this.z;
        end
        
        function linearizeOplus(this)
            this.J{1} = [1 0 0 0;
                0 0 1 0];
        end        
    end
end