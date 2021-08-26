function [terminalStates, times] = exampleHelperBasicLeadVehicleFollow(refPath, laneWidth, safetyGap, egoState, actorState, dt)
%exampleHelperBasicLeadVehicleFollow Generates terminal states for vehicle following behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLeadVehicleFollow(REFPATH, LANEWIDTH, SAFETYGAP, EGOSTATE, ACTORSTATE, DT)
%   Generates terminal states that attempt to follow behind the nearest
%   vehicle in the ego vehicle's current lane over given spans of time, DT.
%
%   REFPATH is a referencePathFrenet object used to convert the ego and 
%   actors' state, EGOSTATE/ACTORSTATE, respectively, from global coordinates, 
%   [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL]. 
%   
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lanes of all actors over the given times, DT. The nearest
%   actor leading the ego vehicle in the same lane is chosen as the vehicle
%   to follow.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: 
%   [(S_lead-SAFETYGAP) dS_lead 0 L_lead dL_lead 0], where *_lead denotes
%   the state of the nearest lead vehicle.
%
% Copyright 2020 The MathWorks, Inc.
    
    % Convert ego state to Frenet coordinates
    frenetStateEgo = global2frenet(refPath, egoState);

    % Get current lane of ego vehicle
    curEgoLane = exampleHelperPredictLane(frenetStateEgo, laneWidth, 0);

    % Get current and predicted lanes for each actor
    frenetStateActors = global2frenet(refPath, actorState);
    
    predictedActorLanes = zeros(numel(dt),size(actorState,1));
    for i = 1:size(actorState,1)
        predictedActorLanes(:,i) = exampleHelperPredictLane(frenetStateActors(i,:),laneWidth,dt);
    end
    % For each time horizon, find the closest car in the same lane as
    % ego vehicle
    terminalStates = zeros(numel(dt),6);
    validTS = false(numel(dt),1);
    for i = 1:numel(dt)
        % Find vehicles in same lane t seconds into the future
        laneMatch = curEgoLane == predictedActorLanes(i,:)';

        % Determine if they are ahead of the ego vehicle
        leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

        % Of these, find the vehicle closest to the ego vehicle (assume
        % constant longitudinal velocity)
        future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
        future_S(~leadVehicle | ~laneMatch) = inf;
        [actor_S1, idx] = min(future_S);

        % Check if any car meets the conditions
        if actor_S1 ~= inf
            % If distance is greater than safety gap, set the terminal
            % state behind this lead vehicle
            if frenetStateEgo(1)+safetyGap < actor_S1
                ego_S1 = actor_S1-safetyGap;
                terminalStates(i,:) = [ego_S1 frenetStateActors(idx,2) 0 frenetStateActors(idx,4:5) 0];
                validTS(i) = true;
            end
        end
    end
    % Remove any bad terminal states
    terminalStates(~validTS,:) = [];
    times = dt(validTS(:));
    times = times(:);
end