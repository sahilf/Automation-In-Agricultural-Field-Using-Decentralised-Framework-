clc;close all;clear all;
Model = differentialDriveKinematics;
initialState=input('Enter the initial state of the robot: ');
initialState(3)=initialState(3)*pi/180;
tspan = 0:0.05:1;
r=Model.WheelRadius;
Vl=input('Enter left wheel velocity: ');
Vr=input('Enter right wheel velocity: ');
inputs = [Vl/r Vr/r]; 
[t,pose] = ode45(@(t,pose)derivative(Model,pose,inputs),tspan,initialState);
i=1;
leng=length(pose(:,1));
for i=1:1:leng
    plot(pose(i,1),pose(i,2),'-bx');
    xlabel('X-axis');
    ylabel('Y-axis');
    plot(initialState(1),initialState(2),'r.','MarkerSize',10);
    axis([-5 5 -5 5]);
    grid on;
    pause(1);
    hold on;
end
disp('Current Pose of the bot')
pose(:,3)=pose(:,3)*180/pi;
disp(pose(end,:))
