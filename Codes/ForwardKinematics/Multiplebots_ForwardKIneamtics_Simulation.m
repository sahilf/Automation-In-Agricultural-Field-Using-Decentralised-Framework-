clc;clear all;close all;
Rob1=differentialDriveKinematics;
Rob2=differentialDriveKinematics;
prompt = {'Enter R1 position:','Enter velocities of R1:','Enter R2 position:','Enter velocities of R2'};
dlgtitle = 'Input';
dims = [1 35];
answer = inputdlg(prompt,dlgtitle,dims);
initial1=str2num(answer{1});
initial1(3)=initial1(3)*pi/180;
input1=str2num(answer{2});
initial2=str2num(answer{3});
initial2(3)=initial2(3)*pi/180;
input2=str2num(answer{4});
r1=Rob1.WheelRadius;
r2=Rob2.WheelRadius;
input1=input1/r1;
input2=input2/r2;
[t1,pose1]=ode45(@(t1,pose1) derivative(Rob1,pose1,input1),[0:0.05:1],initial1);
[t2,pose2]=ode45(@(t2,pose2) derivative(Rob1,pose2,input2),[0:0.05:1],initial2);
len=length(t1);
for i=1:1:len
    plot(pose1(i,1),pose1(i,2),'-bx');
    xlabel('X-axis');
    ylabel('Y-axis');
    plot(initial1(1),initial1(2),'r.','MarkerSize',10);
    text(initial1(1),initial1(2),'R1','VerticalAlignment','bottom','HorizontalAlignment','right');
    hold on;
    plot(pose2(i,1),pose2(i,2),'-bx');
    plot(initial2(1),initial2(2),'r.','MarkerSize',10);
    text(initial2(1),initial2(2),'R2','VerticalAlignment','bottom','HorizontalAlignment','right');
    axis([-5 5 -5 5]);
    grid on;
    pause(1);
%     hold on;
end
hold off;
disp('Current Pose of the R1')
pose1(:,3)=pose1(:,3)*180/pi;
disp(pose1(end,:))
disp('Current Pose of the R2')
pose2(:,3)=pose2(:,3)*180/pi;
disp(pose2(end,:))

