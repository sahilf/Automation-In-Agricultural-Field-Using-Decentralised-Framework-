function DDR(points,numberOfSides)
    robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
    robotCurrentPose = [points(1,:) 0]';
    robotInitialLocation= points(1,:);
    robotGoal = points(end-1,:);
    path = points;
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.25;
    controller.MaxAngularVelocity = 2;
    controller.LookaheadDistance = 0.11;
    goalRadius = 0.2;
    distanceToGoal = norm(robotInitialLocation - robotGoal);
    sampleTime = 0.1;
    vizRate = rateControl(1/sampleTime);
    frameSize = robot.TrackWidth/0.8;
    robotCoveredPosition = [];

    while( distanceToGoal > goalRadius )

        [v, omega] = controller(robotCurrentPose); %controller outputs
        vel = derivative(robot, robotCurrentPose, [v omega]); %robot _velocity (3x1)
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

        hold off

        plot(path(:,1), path(:,2),"b--o",'LineWidth',1.5)
        xlabel('x-axis');
        ylabel('y-axis');
%         xlim auto
%         ylim auto
        %xlim([-(numberOfSides/2),(numberOfSides/2)])
        %ylim([-(numberOfSides/2),(numberOfSides/2)+0.5])
        hold on;

        robotCoveredPosition = [robotCoveredPosition;[robotCurrentPose(1),robotCurrentPose(2)]];
        plot(robotCoveredPosition(:,1), robotCoveredPosition(:,2),"r-",'LineWidth',1.5)


        plot(robotCurrentPose(1,1),robotCurrentPose(2,1),'ro');
        plotTrVec = [robotCurrentPose(1:2); 0];
        plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.15);
        light;
        waitfor(vizRate);
    end
end
