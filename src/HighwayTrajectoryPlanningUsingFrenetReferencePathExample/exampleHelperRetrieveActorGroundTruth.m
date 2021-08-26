function [curState, futureTrajectory, isRunning] = exampleHelperRetrieveActorGroundTruth(scenario, futureTrajectory, replanRate, maxHorizon)
%exampleHelperRetrieveActorGroundTruth Retrieve current and future states of each actor over a given time horizon.
%
%   This function is for internal use only. It may be removed in the future

%   Advances a drivingScenario object, SCENARIO, ahead of the current
%   simulation timestep and stores the states of each actor in
%   FUTURETRAJECTORY. FUTURETRAJECTORY is an M-by-1 struct array containing the
%   field Trajectory, an N-by-6 matrix, where M is the number of non-ego 
%   actors and N is the number of future steps stored by the trajectory. N is 
%   determined using the REPLANRATE of the planner and the maximum timespan
%   over which planned trajectories can occur, MAXHORIZON.
%
%   This function can be replaced by custom prediction modules or external
%   ground truth data.
%
% Copyright 2020 The MathWorks, Inc.
    
    numActor = numel(futureTrajectory);
    curState = zeros(numActor,6);

    % Number of states needed
    minUpdateSteps = (1/replanRate)/scenario.SampleTime;
    maxNumStates = maxHorizon/scenario.SampleTime;
    statesNeeded = max(maxNumStates-size(futureTrajectory(1).Trajectory,1),minUpdateSteps);

    for i = 1:statesNeeded
        % Advance the scenario
        isRunning = advance(scenario);

        % Retrieve actor information
        poses = scenario.actorPoses;
        for j = 1:numActor
            actIdx = j+1;
            xy = poses(actIdx).Position(1:2);
            v  = norm(poses(actIdx).Velocity,2);
            th = atan2(poses(actIdx).Velocity(2),poses(actIdx).Velocity(1));
            k = poses(actIdx).AngularVelocity(3)/v/180*pi;
            % Assume acceleration = 0
            futureTrajectory(j).Trajectory(i,:) = [xy th k v 0];
        end
        if ~isRunning
            break;
        end
    end

    % Reorder the states
    for i = 1:numActor
        futureTrajectory(i).Trajectory = circshift(futureTrajectory(i).Trajectory,-statesNeeded,1);
        curState(i,:) = futureTrajectory(i).Trajectory(1,:);
    end
end