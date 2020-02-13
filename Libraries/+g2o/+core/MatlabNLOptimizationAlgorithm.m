classdef MatlabNLOptimizationAlgorithm < g2o.core.OptimizationAlgorithm
    
    methods(Access = public)
       
        function this = MatlabNLOptimizationAlgorithm()
            this = this@g2o.core.OptimizationAlgorithm();            
        end
        
        function numberOfIterations = solve(this, X0, maximumNumberOfIterations)
            
            % The wrapper function
            graphWrapper = @(X) this.errorFunction(X);
            
            % Optimizer options so we can see what's going on
            options = optimoptions('lsqnonlin', 'Display', 'iter', ...
                'MaxIterations', maximumNumberOfIterations, ...
                'MaxFunctionEvaluations', 100 * maximumNumberOfIterations);
            
            X = lsqnonlin(graphWrapper, X0, [], [], options);
            
            this.optimizableGraph.assignXToVertices(X);
            
            numberOfIterations = 0;
        end
    end
    
    methods(Access = protected)
        
        function chi2Vector = errorFunction(this, X)
            this.optimizableGraph.assignXToVertices(X);
            edges = this.optimizableGraph.edges().values();
            chi2Vector = zeros(1, length(edges));
            for e = 1 : length(edges)
                edges{e}.computeError();
                chi2Vector(e) = sqrt(edges{e}.chi2());
            end
        end
        
    end
    
end