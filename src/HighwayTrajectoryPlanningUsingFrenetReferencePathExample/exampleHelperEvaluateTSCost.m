function costs = exampleHelperEvaluateTSCost(terminalStates, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
%exampleHelperEvaluateTSCost Evaluate trajectory cost.
%exampleHelperEvaluateTSCost计算轨迹成本
%   This function is for internal use only. It may be removed in the future

%   COSTS = exampleHelperEvaluateTSCost(TERMINALSTATES, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight) 
%   Evaluates the cost of an N-by-6 matrix of Frenet states, TERMINALSTATES,
%   and corresponding N-by-1 vector of timespans, TIMES. 
%
%   Cost is based on lateral deviation from a lane center, calculated using
%   LANEWIDTH, deviation from the desired velocity, SPEEDLIMIT, and the span
%   of time required by the trajectory, TIMES. 
%成本基于偏离车道中心的横向偏差，使用laneWidth，偏离预期速度的偏差，speedLimit和轨迹所需的时间跨度times计算
%   Each of these metrics is scaled by the accompanying weights,
%   LATWEIGHT, SPEEDWEIGHT, and TIMEWEIGHT, respectively.
%每一个指标都有相应的权重(speedWeight, latWeight, timeWeight)进行缩放。
% Copyright 2020 The MathWorks, Inc.
    
    % Find lateral deviation from nearest lane center，找出离最近车道中心的横向偏差
    laneCenters = (1.5:-1:-1.5)*laneWidth;
    latDeviation = abs(laneCenters-terminalStates(:,4)); %计算规划的终点横向位置距离四条车道的距离

    % Calculate lateral deviation cost
    latCost = min(latDeviation,[],2)*latWeight;  %取每种规划中距离四条车道最近的值？？？这是什么鬼逻辑？？？不应该是计算每种规划终点距离自车的横向距离吗？

    % Calculate trajectory time cost，计算轨迹的时间成本，优先选择规划时间长的，因为规划的时间跨度越长，肯定越平滑，其实我也不是很能苟同，平滑不平滑应该交给横向加速度等指标去评判，在这里规划的时间越长我觉得越不太好，这是效率的原因。
    timeCost = times*timeWeight;

    % Calculate terminal speed vs desired speed    % cost，计算终端速度和期望速度的偏差cost，离期望速度越近成本越小
    speedCost = abs(terminalStates(:,2)-speedLimit)*speedWeight;

    % Return cumulative cost  %返回总的cost
    costs = latCost+timeCost+speedCost;
end