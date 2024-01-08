clc;clear all;close all;
R=differentialDriveKinematics;
r=R.WheelRadius;
l=R.TrackWidth;
Vr=2;
Vl=2;
initial=input('Enter initial position of the robot: ');
final=input('Enter final position of the robot: ');
d=round(pdist([initial(1),initial(2);final(1),final(2)],'euclidean'),4);
m=(final(2)-initial(2))/(final(1)-initial(1));
o=atand(m)-initial(3);
if d~=0
    %In-place rotation
    if o~=0
        [t,flag]=inplace_rotation(o,Vr,Vl,l);
        inputs=[Vl/r Vr/r];
        if flag
            inputs(1)=-1*inputs(1);
        else
            inputs(2)=-1*inputs(2);
        end
        [time,pose]=solver(R,initial,inputs,t);
        current=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
        motion=linear_motion(current,final,Vr,d/Vr);
        if motion=='r'
            temp=pose;
            %temp(:,3)=temp(:,3)*180/pi;
            if flag
                o=180;
            else
                o=-180;
            end
            [t,flag]=inplace_rotation(o,Vr,Vl,l);
            [time,pose]=solver(R,current,inputs,t);
            current=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
            pose=cat(1,temp,pose);
            disp(pose);
        end
        plot_fun(initial(1),initial(2),pose);
        current=round(current,4);
    else
        current=initial;
    end
    disp('After first in-place rotation');
    disp(current)

    %Linear motion to given x,y position
    t=d/Vr;
    inputs=[Vl/r Vr/r];
    [time,pose]=solver(R,current,inputs,t);
    current=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
    len=length(pose);
    figure;
    for i=1:2:len
        plot(pose(i,1),pose(i,2),'-bx');
        xlabel('X-axis');
        ylabel('Y-axis');
        plot(initial(1),initial(2),'r.','MarkerSize',10);
        axis([-5 5 -5 5]);
        grid on;
        pause(1);
        hold on;
    end
    current=round(current,4);
else
    current=initial;
end
disp('After linear motion');
disp(current)
%Second In-place rotation
b=final(3)-current(3);
if b~=0
    [t,flag]=inplace_rotation(b,Vr,Vl,l);
    inputs=[Vl/r Vr/r];
    if flag
        inputs(1)=-1*inputs(1);
    else
        inputs(2)=-1*inputs(2);
    end
    [time,pose]=solver(R,current,inputs,t);
    current=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
    plot_fun(current(1),current(2),pose);
    current=round(current,4);
end
disp('After second and final in-place rotation');
disp(current)


