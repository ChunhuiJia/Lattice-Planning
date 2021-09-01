function [terminalStates, times] = exampleHelperBasicCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt)
%exampleHelperBasicCruiseControl Generates terminal states for cruise control behavior.
%   exampleHelperBasicCruiseControl 生成巡航控制行为的终端状态。
%   This function is for internal use only. It may be removed in the
%   future，此功能仅供内部使用，将来可能会被删除
%   [TERMINALSTATES, TIMES] = exampleHelperBasicCruiseControl(REFPATH, LANEWIDTH, EGOSTATE, TARGETVELOCITY, DT)
%   Generates terminal states that attempt to obey the speed limit,
%   %生成试图遵守速度限制的终端状态
%   TARGETVELOCITY, while following a lane center over a given spans of time, DT.
%   targetVelocity，在给定的时间跨度(dt)内跟随车道中心。
%   REFPATH is a referencePathFrenet object used to convert the ego 
%   refpath是一个referencePathFrenet对象，用于转换ego。
%   vehicle's state, EGOSTATE, from global coordinates, [x y theta kappa v a], 
%   to Frenet coordinates, [S dS ddS L dL ddL]. 
%   车辆的状态egoState，从全局坐标[x y theta kappa v a]转换到Frenet坐标系[S dS ddS L dL ddL].
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lane that the vehicle would reside in based on the current
%   Frenet state and zero terminal velocity/acceleration over the given 
%   span of time.
%   使用Frenet坐标后，exampleHelperPredictLane用于根据当前Frenet状态和零终端速度/加速度来预测车辆所在的未来车道
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: [NaN TARGETVELOCITY 0 L_lane 0 0], where 
%   L_lane is the lateral deviation of the predicted lane-center. TIMES is 
%   also returned as an N-by-1 vector of doubles.
%   该函数返回terminalStates，一个N*6矩阵，其中每一行是一个Frenet状态，定义如下：[NaN TARGETVELOCITY 0 L_lane 0 0]，
%   其中L_lane是预测车道中心的横向偏差，times也作为N*1的双精度向量返回
% Copyright 2020 The MathWorks, Inc.
    
    % Convert ego state to Frenet coordinates，把自车的状态转换到Frenet坐标系上
    frenetState = global2frenet(refPath, egoState);

    % Determine current and future lanes，确定本车在未来dt个间隔点时所处的车道
    futureLane = exampleHelperPredictLane(frenetState, laneWidth, dt);

    % Convert future lanes to lateral offsets，将未来车道转换为横向偏移，没看明白为什么这样转？？？
    lateralOffsets = (2-futureLane+.5)*laneWidth;

    % Return terminal states
    terminalStates      = zeros(numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = targetVelocity;
    terminalStates(:,4) = lateralOffsets;
    times = dt(:);
end