function lineHandles = exampleHelperVisualizeScene(lineHandles, globalTrajectories, isValid, trajectoryEvaluation)
%exampleHelperVisualizeScene Display the current and predicted scene
%
% Copyright 2020 The MathWorks, Inc.
    
    hold on;

    numTraj = numel(globalTrajectories);

    maxNumTraj = 20;
    if isempty(lineHandles)
        for i = 1:maxNumTraj
            lineHandles(end+1) = line;
        end
    end
    optimalIdx = find(trajectoryEvaluation==1,1);
    if ~isempty(optimalIdx)
        width = 4;
        color = 'g';
        lineStyle = '-';
        traj = globalTrajectories(optimalIdx).Trajectory;
        set(lineHandles(1),'XData',traj(:,1),'YData',traj(:,2),'LineStyle',lineStyle,'Color',color,'LineWidth',width);
    else
        set(lineHandles(1),'XData',[],'YData',[]);
    end
    width = 2;
    idx = 2;
    for i = 1:maxNumTraj
        if i <= numTraj
            traj = globalTrajectories(i).Trajectory;
            if isValid(i)
                lineStyle = '-';
                if ~isnan(trajectoryEvaluation(i))
                    % Trajectory was evaluated
                    if trajectoryEvaluation(i) ~= 1
                        color = 'r';
                    end
                else
                    % Trajectory did not get evaluated
                    color = 'w';
                end
            else
                % Trajectory violated a constraint, and therefore was never
                % evaluated
                lineStyle = '--';
                color = 'c';
            end
            if isempty(optimalIdx) || i ~= optimalIdx
                set(lineHandles(idx),'XData',traj(:,1),'YData',traj(:,2),'LineStyle',lineStyle,'Color',color,'LineWidth',width);
                idx = idx+1;
            end
        else
            set(lineHandles(idx),'XData',[],'YData',[]);
            idx = idx+1;
        end
    end
    hold off;
    drawnow
end