% This class models a linear process model. The process model is
%
% x(k+1)=F*x(k)+v(k)
%
% and v(k) is a Random Gaussian noise term
%
% The implementation is deliberately incomplete for Task 2.

classdef ObjectProcessModelEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        
        % The state transition matrix
        F;
        
    end
    
    methods(Access = public)
        function this = ObjectProcessModelEdge()
            this = this@g2o.core.BaseBinaryEdge(4);   
            
            % Create the state transtion matrix. Set it to NaN to trigger
            % errors if we don't assign a value to it.
            this.F = NaN(4, 4);            
        end
        
        function setF(this, F)
            this.F = F;
        end
       
        % This method needs to be implemented for Task 2
        function computeError(this)
            %this.errorZ = NaN(4, 1);
            this.errorZ = this.F * this.edgeVertices{1}.x ...
                - this.edgeVertices{2}.x;
        end
        
        % This method needs to be implemented for Task 2
        function linearizeOplus(this)
            %this.J{1} = NaN(4, 4);
            %this.J{2} = NaN(4, 4);
            this.J{1} = this.F;
            this.J{2} = - eye(4);
        end        
        
    end
    
end