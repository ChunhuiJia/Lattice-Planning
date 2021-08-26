function costs = exampleHelperEvaluateTSCost(terminalStates, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
%exampleHelperEvaluateTSCost Evaluate trajectory cost.
%
%   This function is for internal use only. It may be removed in the future

%   COSTS = exampleHelperEvaluateTSCost(TERMINALSTATES, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight) 
%   Evaluates the cost of an N-by-6 matrix of Frenet states, TERMINALSTATES,
%   and corresponding N-by-1 vector of timespans, TIMES. 
%
%   Cost is based on lateral deviation from a lane center, calculated using
%   LANEWIDTH, deviation from the desired velocity, SPEEDLIMIT, and the span
%   of time required by the trajectory, TIMES. 
%   
%   Each of these metrics is scaled by the accompanying weights,
%   LATWEIGHT, SPEEDWEIGHT, and TIMEWEIGHT, respectively.
%
% Copyright 2020 The MathWorks, Inc.
    
    % Find lateral deviation from nearest lane center
    laneCenters = (1.5:-1:-1.5)*laneWidth;
    latDeviation = abs(laneCenters-terminalStates(:,4));

    % Calculate lateral deviation cost
    latCost = min(latDeviation,[],2)*latWeight;

    % Calculate trajectory time cost
    timeCost = times*timeWeight;

    % Calculate terminal speed vs desired speed cost
    speedCost = abs(terminalStates(:,2)-speedLimit)*speedWeight;

    % Return cumulative cost
    costs = latCost+timeCost+speedCost;
end