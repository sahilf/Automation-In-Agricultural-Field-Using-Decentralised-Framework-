close all;clc;
bot=1;
path=path_points{bot};
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = org_pos(bot,3)*pi/180;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
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
plot(path(:,1), path(:,2),"b--o")
hold on;
plot(vorvx{bot}(:,1),vorvx{bot}(:,2),'k')
xlim([min(path(:,1))-0.5,max(path(:,1))+0.5])
ylim([min(path(:,2))-0.5,max(path(:,2))+0.5])
xlabel('x-axis');
ylabel('y-axis');
title('Movement of R1 in its area');
plotTrVec = [robotCurrentPose(1:2); 0];
plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
light;

disp('Establishing Communication.....');
pause(3);
t2 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
fopen(t2);
t1 = tcpclient('localhost', 30000);
disp('Communication Established.');
disp('Transfer of path data is in process.....');
shared_path =read1_new(t2,1);
write1_new(t1,path,1);
pause(11);
Global_timer = 0;
comms=1;
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
    xlabel('x-axis');
    ylabel('y-axis');
    title('Movement of R1 in its area');
    hold all
    robotCurrentPose =round(robotCurrentPose,4);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
    light;
    stop_timer = toc(start_timer);
    Global_timer = Global_timer + stop_timer +0.1;
    if Global_timer >20
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
        light;
%         read
            if comms
                [data,comms]=read1_new(t2,0);
                if comms
                    prev_data = data;
                end
                disp('Received Data');
                disp(data);
%                 write
                write1_new(t1,robotCurrentPose,0);
            end
        Global_timer = 0;
    end
    waitfor(vizRate); 
end

% shared_path = path_points{2};
prev_data = prev_data(1:2,:);
coordinate_values = [];
for i=1:2
    if prev_data(i)<1
        coordinate_values = [coordinate_values ceil(prev_data(i)*10)/10];
        coordinate_values = [coordinate_values floor(prev_data(i)*10)/10];
    else
        coordinate_values = [coordinate_values ceil(prev_data(i))];
        coordinate_values = [coordinate_values floor(prev_data(i))-1];
    end
end
possibilities =[];
for i=1:2
    possibilities = [possibilities ;[coordinate_values(i) coordinate_values(3)]];
    possibilities = [possibilities ;[coordinate_values(i) coordinate_values(4)]];
end
for i=1:length(possibilities)
    for j=1:length(shared_path)
        if isequal(shared_path(j,:),possibilities(i,:))
            other_bot_pos = j;
            break
        end
    end
end
% disp(other_bot_pos);
path_to_track = shared_path(other_bot_pos+4:end,:);
% disp(path_to_track)

% robotCurrentPose = [0;3;0];
robotCurrentPose(3) = 0;
% path_to_track = [];
% points =  [3 6 7 10 11];
% for i =1:length(points)
%     path_to_track = [path_to_track;[shared_path(points(i),:)]];
% end
% path_to_track = [path_to_track ;shared_path(points(end)+1:end,:)];
path = shared_path;
controller.Waypoints = path_to_track;
robotGoal = path_to_track(end,:);
distanceToGoal = norm([robotCurrentPose(1),robotCurrentPose(2)] - robotGoal);
count =1;
controller.LookaheadDistance = 0.11;
pause(5);
while( distanceToGoal > goalRadius)   
    
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
    plot(path(1:other_bot_pos+4,1), path(1:other_bot_pos+4,2),"r--o")
    xlabel('x-axis');
    ylabel('y-axis');
    title("Movement of R1 in R2's area");
    hold all
    robotCurrentPose = round(robotCurrentPose,4);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.1);
    light;
    waitfor(vizRate); 
end
