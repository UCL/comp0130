% This is the basic element which is stored in a hypergraph. It basically
% stores just an ID, an optional name, and information to show if it has
% been inserted into a graph.
%
% It should not be instantiated directly.


classdef HyperGraphElement < handle
    
    
    properties(Access = protected)
        
        % Inserted in a graph?
        registeredInGraph;
        
        % The id of the vertex. This is of type int64
        elementId;
        
        % The name of this object
        elementName;
    end
    
    methods(Access = public)
        
        function this = HyperGraphElement()
            this.registeredInGraph = false;
        end
        
    end
    
    methods(Access = public, Sealed = true)
        
        % Get the name of the vertex
        function name = name(this)
            name = this.elementName;
        end
        
        % Get the numerical ID of the vertex
        function id = id(this)
            id = this.elementId;
        end
        
        % Registered with a graph already?
        function registeredInGraph = registered(this)
            registeredInGraph = this.registeredInGraph;
        end
    end
    
    methods(Access = public, Abstract)
        
        % Make sure the element is valid. The definition of validity is
        % different for vertices and edges.
        validate(this);
        
    end
    
    methods(Access = protected)
        
        % Set the ID of the vertex; you can only do this if the vertex has
        % not been registered with a graph
        function setId(this, newElementId)
            
            % Assume we can cast okay
            newElementId = int64(newElementId);
            
            % If no ID has been assigned, we can assume the vertex hasn't
            % been registered with a graph
            if (isempty(this.elementId) == true)
                this.elementId = newElementId;
                return;
            end
            
            % Check we haven't been registered with a graph already
            assert((this.registered == false), ...
                'g2o::hypergraphelement::changeidafterregistration', ...
                ['Attempt to change the id of an element that has ' ...
                'already been added to a graph; oldID=%d, newID=%d'], ...
                this.elementId, newElementId);
            
            % Now change the vertex ID
            this.elementId = newElementId;
            
        end
    end
    
    % These methods are only accessible by the hypergraph
    
    methods(Access = {?g2o.core.HyperGraph})
        
        function setRegistered(this, newRegistered)
            this.registeredInGraph = newRegistered;
        end
    end
end