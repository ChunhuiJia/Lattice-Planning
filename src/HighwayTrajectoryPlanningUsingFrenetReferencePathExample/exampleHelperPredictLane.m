function laneNum = exampleHelperPredictLane(frenetState, laneWidth, dt)
%exampleHelperPredictLane Predicts a vehicles lane for a given set of times
%exampleHelperPredictLane 预测给定时间的车辆车道
%   This function is for internal use only. It may be removed in the future

%   LANENUM = exampleHelperPredictLane(FRENETSTATE, LANEWIDTH, DT) predicts
%   a vehicle's lane over one or more times, DT, in the future based on its 
%   current FRENETSTATE, a 1-by-6 vector in Frenet coordinates [S dS ddS L dL ddL], 
%   and the following assumptions:
%   根据当前的frenetState（Frenet坐标中的1*6向量[S dS ddS L dL ddL]）和以下假设，预测未来车辆的车道一次或者多次DT
%       1) Bang bang acceleration control，加速度控制
%       2) Constant change in lateral acceleration for two control sections，两个控制部分的横向加速度恒定变化
%   	3) Terminal lateral velocity = 0，终端横向速度=0
%       4) Terminal lateral acceleration = 0，终端横向加速度=0
%       5) Scenario occurs on a 4-lane highway with fixed lane width,LANEWIDTH.场景发生在车道宽度固定的4车道高速公路上
%
%           NOTE: When DT is zero, no motion model assumptions are applied.当DT是0时，没有应用运动模型假设
%
% Copyright 2020 The MathWorks, Inc.
    
    narginchk(3,3);

    laneBounds = [inf (2:-1:-2)*laneWidth -inf];
    laneNum = zeros(numel(dt),1);
    
    for i = 1:numel(dt)
        if dt(i) == 0
            dLaneEgo = laneBounds-frenetState(4);
            laneNum(i) = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),4);
        else
            % Retrieve current velocity/acceleration/time,检索当前速度/加速度/时间
            t  = dt(i);
            a0 = frenetState(6);
            v0 = frenetState(5);

            % Solve for the constant change in acceleration and time of application that arrest the ego vehicle's lateral
            % velocity and acceleration over a given number of seconds.
            % 求解在给定的时间内，使自车的横向速度和加速度停止的加速度和应用时间的恒定变化
            if a0 == 0
                avgAcc = -v0/t;
                Ldiff = v0*t + avgAcc/2*t^2;
            else
                a = a0;
                b = (-2*v0-2*a0*t);
                c = (v0*t+a0/2*t^2);

                % Possible time switches
                r = (-b+(sqrt(b^2-4*a*c).*[-1 1]))/(2*a);

                % Select the option that occurs in the future
                rS = r(r>0 & r <= t);

                % Calculate the constant change in acceleration
                da0 = a0/(t-2*rS);

                % Predict total distance traveled over t seconds
                Ldiff = v0*t + a0/2*t^2 + da0/6*t^3 - da0/6*(t-rS)^3;
            end
            % Find distance between predicted offset and each lane，
            %找出预测偏移量和每个车道之间的距离，以自车为横向坐标原点，得到各个车道线相对于本车的横向位置坐标
            dLaneEgo = laneBounds-(frenetState(4)+Ldiff);

            % Determine future lane，确定本车处在第几个车道内
            laneNum(i) = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),4);
            % find():查找非零元素的索引和值
        end
    end
end