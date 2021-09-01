function [terminalStates, times] = exampleHelperBasicLaneChange(refPath, laneWidth, egoState, dt)
%exampleHelperBasicLaneChange Generates terminal states for lane change behavior.
%exampleHelperBasicLaneChange生成换到行为的终端状态
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLaneChange(REFPATH, LANEWIDTH, EGOSTATE, DT)
%   Generates terminal states, TERMINALSTATES, that transition the ego vehicle
%   from the current lane to adjacent lanes. REFPATH is a referencePathFrenet
%   object used to convert the ego vehicle's state, EGOSTATE, from global 
%   coordinates, [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL]. 
%   生成终端状态terminalStates，将本车从当前车道转换到相邻车道
%   refPath是一个referencePathFrenet对象，用于将自车的状态egoState从全局坐标[x y theta kappa v a]转换为 [S dS ddS L dL ddL]
%   Once in Frenet coordinates, exampleHelperPredictLane returns the current
%   lane, and the centers of the adjacent lanes are then calculated relative to
%   REFPATH assuming a fixed LANEWIDTH and 4-lane road.
%   使用Frenet坐标后，exampleHelperPredictLane返回当前车道，然后假设固定的laneWidth和4车道道路，相对于refPath计算相邻车道的中心
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row 
%   is a Frenet state defined as follows: [NaN dS_cur 0 L_lane 0 0], where 
%   dS_cur is the ego vehicle's current longitudinal velocity and L_lane is
%   the lateral deviation of the lane-center. TIMES is also returned as an 
%   N-by-1 vector of doubles.
%   该函数返回terminalStates，一个N*6矩阵，其中每一行是一个Frenet状态，定义为：[NaN dS_cur 0 L_lane 0 0]
%   其中dS_cur是本车当前的纵向速度，L_lane是车道的横向偏差中心，times也作为N*1的双精度向量返回
% Copyright 2020 The MathWorks, Inc.
    
    if egoState(5) == 0
        terminalStates = [];
        times = [];
    else
        % Convert ego state to Frenet coordinates，将自车状态转换为Frenet坐标
        frenetState = global2frenet(refPath, egoState);

        % Get current lane
        curLane = exampleHelperPredictLane(frenetState, laneWidth, 0);

        % Determine if future lanes are available，判断预测的未来车道是否可用，怎么判断？
        adjacentLanes = curLane+[-1 1];
        validLanes = adjacentLanes > 0 & adjacentLanes <= 4;  %判断本车左右车道是否有效

        % Calculate lateral deviation for adjacent lanes，计算相邻车道的横向偏差
        lateralOffset = (2-adjacentLanes(validLanes)+.5)*laneWidth;
        numLane = nnz(validLanes);  % N = nnz(X) 返回矩阵 X 中的非零元素数

        % Calculate terminal states，计算终端状态
        terminalStates = zeros(numLane*numel(dt),6);
        terminalStates(:,1) = nan;  % 不限制位置s
        terminalStates(:,2) = egoState(5);  %末状态车速和换道前的状态的车速相同
        terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
        times = repmat(dt(:),numLane,1);
        %末状态纵向和横向加速度都为0，末状态横向速度设为0
    end
end