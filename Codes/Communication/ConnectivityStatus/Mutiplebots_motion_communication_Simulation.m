clc;clear all;close all;
Rob1=differentialDriveKinematics;
Rob2=differentialDriveKinematics;
Rob3=differentialDriveKinematics;
pos=[[0 0 0];[0 -1 0];[0 1 0]];
input=[2 2];
r1=Rob1.WheelRadius;
input=input/r1;
[t1,pose1]=ode45(@(t1,pose1) derivative(Rob1,pose1,input),[0:0.05:1],pos(1,:));
[t2,pose2]=ode45(@(t2,pose2) derivative(Rob1,pose2,input),[0:0.05:1],pos(2,:));
[t3,pose3]=ode45(@(t3,pose3) derivative(Rob3,pose3,input),[0:0.05:1],pos(3,:));
connectivity=[[0 1 1];[1 0 1];[1 1 0]];
for i=1:3
    disp(strcat('Connection status of R',num2str(i)));
    disp(connectivity(i,:));
end
len=length(t1);
for i=1:1:len
    %plot for R1
    plot(pose1(i,1),pose1(i,2),'-bx');
    xlabel('X-axis');
    ylabel('Y-axis');
    plot(pos(1,1),pos(1,2),'r.','MarkerSize',10);
    text(pos(1,1),pos(1,2),'R1','VerticalAlignment','bottom','HorizontalAlignment','right');
    hold on;
    %plot for R2
    plot(pose2(i,1),pose2(i,2),'-bx');
    plot(pos(2,1),pos(2,2),'r.','MarkerSize',10);
    text(pos(2,1),pos(2,2),'R2','VerticalAlignment','bottom','HorizontalAlignment','right');
    %plot for R3
    plot(pose3(i,1),pose3(i,2),'-bx');
    plot(pos(3,1),pos(3,2),'r.','MarkerSize',10);
    text(pos(3,1),pos(3,2),'R3','VerticalAlignment','bottom','HorizontalAlignment','right');
    axis([-5 5 -5 5]);
    grid on;
    if t1(i)==0.5
        plot(pose1(i,1),pose1(i,2),'r.','MarkerSize',10)
        plot(pose2(i,1),pose2(i,2),'r.','MarkerSize',10)
        plot(pose3(i,1),pose3(i,2),'r.','MarkerSize',10)
        circle(pose1(i,1),pose1(i,2),1.2,'g');
        pause(1)
        circle(pose2(i,1),pose2(i,2),1.2,'c');
        pause(1)
        circle(pose3(i,1),pose3(i,2),1.2,'m');
        pause(1);
        pose_values=[pose1(i,1) pose1(i,2);pose2(i,1) pose2(i,2);pose3(i,1) pose3(i,2)];
        for j=1:3
            connectivity(j,:)=rnetwork_check(pose_values(j,:),pose_values,1.2,connectivity(j,:),j);
        end
        disp('After R Network :');
        for m=1:3
            disp(strcat('Connection status of R',num2str(m)));
            disp(connectivity(m,:));
        end
    end
    pause(1);
end
hold off;
disp('Current Pose of the R1')
pose1(:,3)=pose1(:,3)*180/pi;
disp(pose1(end,:))
disp('Current Pose of the R2')
pose2(:,3)=pose2(:,3)*180/pi;
disp(pose2(end,:))
disp('Current Pose of the R3')
pose3(:,3)=pose3(:,3)*180/pi;
disp(pose3(end,:));


