function [terminalStates, times] = exampleHelperBasicLaneChange(refPath, laneWidth, egoState, dt)
%exampleHelperBasicLaneChange Generates terminal states for lane change behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLaneChange(REFPATH, LANEWIDTH, EGOSTATE, DT)
%   Generates terminal states, TERMINALSTATES, that transition the ego vehicle
%   from the current lane to adjacent lanes. REFPATH is a referencePathFrenet
%   object used to convert the ego vehicle's state, EGOSTATE, from global 
%   coordinates, [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL]. 
%   
%   Once in Frenet coordinates, exampleHelperPredictLane returns the current
%   lane, and the centers of the adjacent lanes are then calculated relative to
%   REFPATH assuming a fixed LANEWIDTH and 4-lane road.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: [NaN dS_cur 0 L_lane 0 0], where 
%   dS_cur is the ego vehicle's current longitudinal velocity and L_lane is
%   the lateral deviation of the lane-center. TIMES is also returned as an 
%   N-by-1 vector of doubles.
%
% Copyright 2020 The MathWorks, Inc.
    
    if egoState(5) == 0
        terminalStates = [];
        times = [];
    else
        % Convert ego state to Frenet coordinates
        frenetState = global2frenet(refPath, egoState);

        % Get current lane
        curLane = exampleHelperPredictLane(frenetState, laneWidth, 0);

        % Determine if future lanes are available
        adjacentLanes = curLane+[-1 1];
        validLanes = adjacentLanes > 0 & adjacentLanes <= 4;

        % Calculate lateral deviation for adjacent lanes
        lateralOffset = (2-adjacentLanes(validLanes)+.5)*laneWidth;
        numLane = nnz(validLanes);

        % Calculate terminal states
        terminalStates = zeros(numLane*numel(dt),6);
        terminalStates(:,1) = nan;
        terminalStates(:,2) = egoState(5);
        terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
        times = repmat(dt(:),numLane,1);
    end
end