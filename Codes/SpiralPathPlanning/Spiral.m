pos = [0 0 ;    % startpoint
       10 10 ] ;  % endpoint
nturns = 5 ;    % number of turns (integer value)
dp = diff(pos,1,1) ;    
R = hypot(dp(1), dp(2)) ;
phi0 = atan2(dp(2), dp(1)) ;
phi = linspace(0, nturns*2*pi, 10000) ; % 10000 = resolution
r = linspace(0, R, numel(phi)) ;
x = pos(1,1) + r .* cos(phi + phi0) ;
y = pos(1,2) + r  .* sin(phi + phi0) ;
% plot(x,y,'b-') ; % nturns crossings, including end point
robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
robotCurrentPose = [pos(1,:) 0]';
robotInitialLocation= pos(1,:);
robotGoal = pos(2,:);
path = [x' y'];
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.7;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.11;
goalRadius = 0.2;
distanceToGoal = norm(robotInitialLocation - robotGoal);
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

frameSize = robot.TrackWidth/0.8;
flag=1;
a = [];
while( distanceToGoal > goalRadius )
    
    [v, omega] = controller(robotCurrentPose); %controller outputs
    
    vel = derivative(robot, robotCurrentPose, [v omega]); %robot _velocity (3x1)
    
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
   
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    hold off
    
    plot(path(:,1), path(:,2))
    xlabel('x-axis');
    ylabel('y-axis');
    title('Motion of R1 in the Allocated Area');
    hold on;
    
    a = [a;[robotCurrentPose(1) robotCurrentPose(2)]];
    hold all
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 2);
    light;
    
    waitfor(vizRate);
end

% r = 5; %outer radius
% a = 1;    %inner radius
% b = 2; %incerement per rev % Jos: changed to see the spiral!!
% n = (r - a)./(b); %number  of revolutions
% th = 2*n*pi;      %angle  
% Th = linspace(0,th,1000);  
% % x = (a + b.*Th/(2*pi)).*cos(Th);
% % y = (a + b.*Th/(2*pi)).*sin(Th);
% % better:
% i = linspace(0,n,1250*720);
% x = (a+b*i).* cos(2*pi*i);
% y = (a+b*i).* sin(2*pi*i);
% plot(x,y)
