function [curState, futureTrajectory, isRunning] = exampleHelperRetrieveActorGroundTruth(scenario, futureTrajectory, replanRate, maxHorizon)
%exampleHelperRetrieveActorGroundTruth Retrieve current and future states of each actor over a given time horizon.
% exampleHelperRetrieveActorGroundTruth在给定的时间范围内检索每个actor的当前和未来状态
%   This function is for internal use only. It may be removed in the
%   future，此功能仅供内部使用，将来可能会被删除

%   Advances a drivingScenario object, SCENARIO, ahead of the current
%   simulation timestep and stores the states of each actor in FUTURETRAJECTORY
%   在当前模拟时间步之前推进一个drivingScenario对象SCENARIO，并将每个acotr的状态存储在FUTURETRAJECTORY
%   FUTURETRAJECTORY is an M-by-1 struct array containing the
%   field Trajectory, an N-by-6 matrix, where M is the number of non-ego 
%   actors and N is the number of future steps stored by the trajectory. 
%   FUTURETRAJECTORY是一个包含字段Trajectory的M×1结构数组，一个N*6矩阵，其中M是非自车actors的数量，N是轨迹存储的未来步数。
%   N is determined using the REPLANRATE of the planner and the maximum timespan
%   over which planned trajectories can occur, MAXHORIZON.
%   N是使用规划器的REPLANRATE和规划轨迹可以发生的最大时间跨度MAXHORIZON来确定的
%   This function can be replaced by custom prediction modules or external
%   ground truth data.
%   此函数可以由自定义预测模块或外部真值数据代替
% Copyright 2020 The MathWorks, Inc.
    
    numActor = numel(futureTrajectory);  %numel():返回矩阵的元素个数
    curState = zeros(numActor,6);

    % Number of states needed
    minUpdateSteps = (1/replanRate)/scenario.SampleTime;    %
    maxNumStates = maxHorizon/scenario.SampleTime;    %
    statesNeeded = max(maxNumStates-size(futureTrajectory(1).Trajectory,1),minUpdateSteps);

    for i = 1:statesNeeded
        % Advance the scenario
        isRunning = advance(scenario);  %advance():进行一步驾驶场景模拟，从场景模拟中获取障碍车辆的各状态信息，采样时长为MAXHORIZON
        %在实际自动驾驶项目中，是由预测模块提供这些信息的。
        % Retrieve actor information %检索acotr信息
        poses = scenario.actorPoses;  %得到场景中所有actor的位置、速度、偏航角、角速度等信息，第一个是本车，后面是其他障碍物参与者
        for j = 1:numActor
            actIdx = j+1;
            xy = poses(actIdx).Position(1:2);
            v  = norm(poses(actIdx).Velocity,2);  %norm():向量范数和矩阵范数,在这里也就是横纵向的总速度幅值
            th = atan2(poses(actIdx).Velocity(2),poses(actIdx).Velocity(1));  %计算障碍车的航向角
            k = poses(actIdx).AngularVelocity(3)/v/180*pi; %actor运动车辆的曲率半径，由yawrate=v/R得k=yawrate/v
            % Assume acceleration = 0
            futureTrajectory(j).Trajectory(i,:) = [xy th k v 0];
        end
        if ~isRunning
            break;
        end
    end

    % Reorder the states,重新排序状态，获取当前障碍actor的状态
    for i = 1:numActor   %不知道下面这个循环平移数组有什么用，平移之后还是原来的矩阵啊。。。？？？
        futureTrajectory(i).Trajectory = circshift(futureTrajectory(i).Trajectory,-statesNeeded,1);  %Y = circshift(A,K,dim) 循环将 A 中的值沿维度 dim 平移 K 个位置。输入 K 和 dim 必须为标量。
        curState(i,:) = futureTrajectory(i).Trajectory(1,:);
    end
end