function [terminalStates, times] = exampleHelperBasicLeadVehicleFollow(refPath, laneWidth, safetyGap, egoState, actorState, dt)
%exampleHelperBasicLeadVehicleFollow Generates terminal states for vehicle following behavior.
%exampleHelperBasicLeadVehicleFollow 生成车辆跟随行为的终端状态。
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLeadVehicleFollow(REFPATH, LANEWIDTH, SAFETYGAP, EGOSTATE, ACTORSTATE, DT)
%   Generates terminal states that attempt to follow behind the nearest
%   vehicle in the ego vehicle's current lane over given spans of time, DT.
%生成终端状态，尝试在给定的时间跨度dt内跟随自车当前车道中最近的车辆
%   REFPATH is a referencePathFrenet object used to convert the ego and 
%   actors' state, EGOSTATE/ACTORSTATE, respectively, from global coordinates, 
%   [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL]. 
%refPath是一个referencePathFrenet对象，对于将自车和actors的状态egoState/actorState分别从[x y %theta kappa v a]转换到Frenet坐标[S dS ddS L dL ddL]
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lanes of all actors over the given times, DT. The nearest
%   actor leading the ego vehicle in the same lane is chosen as the vehicle
%   to follow.
%使用Frenet坐标，exampleHelperPredictLane用于预测所有actors在给定时间dt内的未来所处车道。选择在同一车道上引导自车的最近前车作为要跟随的车辆
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: 
%   [(S_lead-SAFETYGAP) dS_lead 0 L_lead dL_lead 0], where *_lead denotes
%   the state of the nearest lead vehicle.
%该函数返回terminalStates，一个N*6矩阵，其中每一行都是一个Frenet状态，定义如下：
%[(S_lead-SAFETYGAP) dS_lead 0 L_lead dL_lead 0]，其中xxx_lead表示最近前车的状态
% Copyright 2020 The MathWorks, Inc.
    % Convert ego state to Frenet coordinates，把本车状态转换到Frenet坐标系中
    frenetStateEgo = global2frenet(refPath, egoState);

    % Get current lane of ego vehicle，获取本车的当前车道
    curEgoLane = exampleHelperPredictLane(frenetStateEgo, laneWidth, 0);

    % Get current and predicted lanes for each actor，把actors转换到Frenet坐标系中
    frenetStateActors = global2frenet(refPath, actorState);
    
    predictedActorLanes = zeros(numel(dt),size(actorState,1));
    for i = 1:size(actorState,1)  %计算各个actors在各给定时间dt所处的车道，会根据当前的acotrs状态进行估算
        predictedActorLanes(:,i) = exampleHelperPredictLane(frenetStateActors(i,:),laneWidth,dt);
    end
    % For each time horizon, find the closest car in the same lane as
    % ego vehicle，对于每个时间范围，找到与自车在同一车道上最近的车辆
    terminalStates = zeros(numel(dt),6);
    validTS = false(numel(dt),1);
    for i = 1:numel(dt)
        % Find vehicles in same lane t seconds into the future，在未来t秒内查找同一车道上的车辆
        laneMatch = curEgoLane == predictedActorLanes(i,:)';

        % Determine if they are ahead of the ego vehicle，确定它们是否位于自车前方
        leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

        % Of these, find the vehicle closest to the ego vehicle (assume
        % constant longitudinal velocity)，找到最接近自车的车辆（假设纵向速度恒定）
        future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
        future_S(~leadVehicle | ~laneMatch) = inf;  
        [actor_S1, idx] = min(future_S);   %在径前车在dt时间的纵向位置s

        % Check if any car meets the conditions，检查是否有汽车符合条件
        if actor_S1 ~= inf
            % If distance is greater than safety gap, set the terminal
            % state behind this lead vehicle，如果距离大于安全间隙，将终端状态设置在前车后面距离前车后方safetyGap处
            if frenetStateEgo(1)+safetyGap < actor_S1  
                ego_S1 = actor_S1-safetyGap;
                terminalStates(i,:) = [ego_S1 frenetStateActors(idx,2) 0 frenetStateActors(idx,4:5) 0];
                %末状态的速度和前车保持一直，横向位置和速度和前车保持一致，横纵向加速度=0
                validTS(i) = true;
            end
        end
    end
    % Remove any bad terminal states
    terminalStates(~validTS,:) = [];  %和前车的距离小于safetyGap，则认为无效，不进行跟车规划（不给出这种情况下的终端状态）
    times = dt(validTS(:));
    times = times(:);
end