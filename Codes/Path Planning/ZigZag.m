close all;clc;
bot=1;
path=path_points{bot};
robotInitialLocation = org_pos(bot,1:2);
robotGoal = path(end,:);
initialOrientation = org_pos(bot,3)*pi/180;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
figure
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.11;
goalRadius = 0;
distanceToGoal = norm(robotInitialLocation - robotGoal);
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

frameSize = robot.TrackWidth/0.8;
flag=1;
a = [];
while( distanceToGoal > goalRadius )
% while 1
    
    [v, omega] = controller(robotCurrentPose); %controller outputs
    
    vel = derivative(robot, robotCurrentPose, [v omega]); %robot _velocity (3x1)
    
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
%     if robotCurrentPose(1:2)==robotGoal
%         break;
%     end
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
%     disp(distanceToGoal)
    
    hold off
    
    plot(path(:,1), path(:,2),"b--o",'LineWidth',1.5)
    xlabel('x-axis');
    ylabel('y-axis');
    title('Motion of R1 in the Allocated Area');
    hold on;
    plot(vorvx{bot}(:,1),vorvx{bot}(:,2),'k')
    xlim([min(path(:,1))-0.5,max(path(:,1))+0.5])
    ylim([min(path(:,2))-0.5,max(path(:,2))+0.5])

%     plot(robotCurrentPose(1),robotCurrentPose(2),'r.');
    a = [a;[robotCurrentPose(1) robotCurrentPose(2)]];
    hold all
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
    light;
    
    waitfor(vizRate);
end