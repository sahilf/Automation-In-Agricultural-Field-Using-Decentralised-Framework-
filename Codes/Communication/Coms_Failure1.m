close all;clc;
bot=1;
path=path_points{bot};
robotInitialLocation = path(1,:);
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
goalRadius = 0.05;
distanceToGoal = norm(robotInitialLocation - robotGoal);
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);


frameSize = robot.TrackWidth/0.8;
flag=1;
Global_timer = 0 ;
established =0;
% t2 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
% fopen(t2);
% t1 = tcpclient('localhost', 30000);
% read1(t2,1);
% disp(path);
% write1(t1,path,1);
coms = 0;
while( distanceToGoal > goalRadius)
    start_timer = tic;    
    [v, omega] = controller(robotCurrentPose); %controller outputs
    
    vel = derivative(robot, robotCurrentPose, [v omega]); %robot _velocity (3x1)
    
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    hold off
    
    plot(path(:,1), path(:,2),"b--o")
    hold on;
    plot(vorvx{bot}(:,1),vorvx{bot}(:,2),'k')
    xlim([min(path(:,1))-0.5,max(path(:,1))+0.5])
    ylim([min(path(:,2))-0.5,max(path(:,2))+0.5])
    hold all
    robotCurrentPose =round(robotCurrentPose,4);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
    light;

    stop_timer = toc(start_timer);
    Global_timer = Global_timer + stop_timer +0.1;
    if Global_timer >20
%         plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
%         light;
        if coms
            %read
            read1(t2,0);

            %write
            write1(t1,robotCurrentPose,0);

            Global_timer = 0;
        end
    end
    waitfor(vizRate); 
end
path = path_points{2};
controller.Waypoints = path_points{2};
robotGoal = path_points{2}(end,:);
distanceToGoal = norm([robotCurrentPose(1),robotCurrentPose(2)] - robotGoal);
while( distanceToGoal > goalRadius)
    start_timer = tic;    
    [v, omega] = controller(robotCurrentPose); %controller outputs
    
    vel = derivative(robot, robotCurrentPose, [v omega]); %robot _velocity (3x1)
    
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    hold off
    
    plot(path(:,1), path(:,2),"b--o")
    hold on;
    plot(vorvx{2}(:,1),vorvx{2}(:,2),'k')
    xlim([min(path(:,1))-0.5,max(path(:,1))+0.5])
    ylim([min(path(:,2))-0.5,max(path(:,2))+0.5])
    hold all
    robotCurrentPose =round(robotCurrentPose,4);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
    light;

    stop_timer = toc(start_timer);
    Global_timer = Global_timer + stop_timer +0.1;
    if Global_timer >20
%         plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
%         light;
        if coms
            %read
            read1(t2,0);

            %write
            write1(t1,robotCurrentPose,0);

            Global_timer = 0;
        end
    end
    waitfor(vizRate); 
end

