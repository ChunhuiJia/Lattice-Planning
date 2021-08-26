function [viewer, futureTrajectory, actorID, actorPoses, egoID, egoPoses, stepPerUpdate, egoState, isRunning, lineHandles] = ...
    exampleHelperInitializeSimulator(scenarioObj, capList, refPath, laneWidth, replanRate, carLen)
%exampleHelperInitializeSimulator Set up scenario and scenario viewer
%
% Copyright 2020 The MathWorks, Inc.
    
    % Reset the scenario
    restart(scenarioObj)
    close all;
    % Create storage container for predicted actor trajectories
    numActors = numel(scenarioObj.Actors)-1;
    futureTrajectory = repelem(struct('Trajectory',[]),numActors,1);

    % Place the ego vehicle at the start of the reference path
    egoState = frenet2global(refPath,[0 0 0 -0.5*laneWidth 0 0]);

    % Retrieve pose struct from dynamicCapsuleList for all actors and ego car
    [actorID, actorPoses] = obstaclePose(capList, capList.ObstacleIDs);
    [egoID, egoPoses] = egoPose(capList, capList.EgoIDs);

    stepPerUpdate = (1/replanRate)/scenarioObj.SampleTime;
    
    % Create a second scenario object. This will be used to visualize the
    % scenario in real time.
    viewer = drivingScenarioTrafficExample;
    viewer.SampleTime = scenarioObj.SampleTime;
    
    % Create a chasePlot using the visualization scenario.
    chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3, 0],'ViewHeight',10,'ViewPitch',20);

    % Create a pop-out figure and display the current scene.
    f = gcf;
    f.Visible = 'on';
    advance(viewer);
    annotationSpacing = 0.05;
    a = [];
    x = [.9 .95];
    y = [.95 .95];
    bdims = [x(1)-.175 ...
             y(1)-(annotationSpacing*3.5) ...
             (x(2)-x(1))+.2 ...
             annotationSpacing*4];
    p = uipanel(f,'Position',bdims,'BackgroundColor',[.75 .75 .75]);
    commonProps = {'VerticalAlignment','middle','HorizontalAlignment','center','Margin',0,'FontUnits','normalized','Color','k'};
    a = annotation(p,'textbox', [.05 .725 .9 .15],'String','Optimal  ',            'EdgeColor','g','BackgroundColor','g','LineWidth',5, commonProps{:});
    a2 = annotation(p,'textbox',[.05 .525 .9 .15],'String','Colliding  ',          'EdgeColor','r','BackgroundColor','r','LineWidth',2, commonProps{:});
    a3 = annotation(p,'textbox',[.05 .325 .9 .15],'String','NotEvaluated  ',      'EdgeColor','w','BackgroundColor','w','LineWidth',2, commonProps{:});
    a4 = annotation(p,'textbox',[.05 .125 .9 .15],'String','ConstraintViolated  ','EdgeColor','c','BackgroundColor','c','LineWidth',2, 'LineStyle','--',commonProps{:});
    a.FontSize  = .1;
    a2.FontSize = .1;
    a3.FontSize = .1;
    a4.FontSize = .1;
    
    lineHandles = [];
    isRunning = true;
    
    ax = findobj(f,'Type','Axes');
    ax.ZLim = [-100 100];