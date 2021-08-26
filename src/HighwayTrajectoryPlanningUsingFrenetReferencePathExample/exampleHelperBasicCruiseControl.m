function [terminalStates, times] = exampleHelperBasicCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt)
%exampleHelperBasicCruiseControl Generates terminal states for cruise control behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicCruiseControl(REFPATH, LANEWIDTH, EGOSTATE, TARGETVELOCITY, DT)
%   Generates terminal states that attempt to obey the speed limit, 
%   TARGETVELOCITY, while following a lane center over a given spans of time, DT.
%
%   REFPATH is a referencePathFrenet object used to convert the ego 
%   vehicle's state, EGOSTATE, from global coordinates, [x y theta kappa v a], 
%   to Frenet coordinates, [S dS ddS L dL ddL]. 
%   
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lane that the vehicle would reside in based on the current
%   Frenet state and zero terminal velocity/acceleration over the given 
%   span of time.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: [NaN TARGETVELOCITY 0 L_lane 0 0], where 
%   L_lane is the lateral deviation of the predicted lane-center. TIMES is 
%   also returned as an N-by-1 vector of doubles.
%
% Copyright 2020 The MathWorks, Inc.
    
    % Convert ego state to Frenet coordinates
    frenetState = global2frenet(refPath, egoState);

    % Determine current and future lanes
    futureLane = exampleHelperPredictLane(frenetState, laneWidth, dt);

    % Convert future lanes to lateral offsets
    lateralOffsets = (2-futureLane+.5)*laneWidth;

    % Return terminal states
    terminalStates      = zeros(numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = targetVelocity;
    terminalStates(:,4) = lateralOffsets;
    times = dt(:);
end