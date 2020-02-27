function results = mainLoop(eventGenerator, localizationSystems)

% This function runs the main loop for the MiniSLAM system. It sets up the
% event generator and the localization systems. It sets up the graphics. It
% runs the main loop and collects results.

% Set up the graphics for output
graphicalOutput = minislam.graphics.GraphicalOutput();
graphicalOutput.initialize(eventGenerator, localizationSystems);

% Helper to make passing arguments easier
if (iscell(localizationSystems) == false)
    localizationSystems = {localizationSystems};
end

% Get the number of localization systems
numLocalizationSystems = length(localizationSystems);

% Allocate the results structure
results = cell(numLocalizationSystems, 1);
for l = 1 : numLocalizationSystems
    results{l} = minislam.Results();
end

% Start the event generator
eventGenerator.start();

storeCount = 0;

while (eventGenerator.keepRunning() == true)
    
    % Get the step number
    storeCount = eventGenerator.stepCount() + 1;
    
    % Print out
    if (rem(storeCount, 100) == 0)
        disp(num2str(storeCount))
    end
    
    % Get the events and generate the events
    events = eventGenerator.events();
    
    % Log the ground truth
    groundTruthState = eventGenerator.groundTruth(false);
    
    for l = 1 : numLocalizationSystems
        localizationSystem = localizationSystems{l};    
        localizationSystem.processEvents(events);
        runOptimizer = localizationSystem.recommendOptimization();
    
        if (runOptimizer == true)
            tic
            localizationSystem.optimize();
            results{l}.optimizationTimes(storeCount) = toc;
        else
            results{l}.optimizationTimes(storeCount) = NaN;
        end
        
        % Store ground truth in each results structure
        results{l}.vehicleTrueStateTime(storeCount) = eventGenerator.time();
        results{l}.vehicleTrueStateHistory(:, storeCount) = groundTruthState.xTrue;

    end
    
    % Draw the graphics. Note this draws once every third frame which is very smooth but
    % can be slow. The graphics only update when "true" is passed in.
    if (runOptimizer == true)%if (((stepNumber - lastUpdateStepNumber) > 10) && (runOptimizer == true))
        graphicalOutput.update();
    %    lastUpdateStepNumber = stepNumber;
    end
    
    eventGenerator.step();
end

% Run the optimizer
for l = 1 : numLocalizationSystems
    localizationSystems{l}.optimize(20);
    [T, X, P] = localizationSystems{l}.robotEstimateHistory();
    results{l}.vehicleStateTime = T;
    results{l}.vehicleStateHistory = X;
    results{l}.vehicleCovarianceHistory = P;
end

graphicalOutput.update();


end