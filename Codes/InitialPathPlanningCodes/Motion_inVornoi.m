clc;clearvars -except org_pos vorvx;close all;
bot=inputdlg('Enter Robot number','Robot',[1 50]);
bot=str2num(bot{1});
R1=struct();
R1.initial_pos=org_pos(bot,:);
R1.model=differentialDriveKinematics;
r=R1.model.WheelRadius;
l=R1.model.TrackWidth;
Vr=2;
Vl=2;
for i=1:length(vorvx)
    vorvx{i}=round(vorvx{i},4);
end
R1.cur_pos=[];
for i=1:length(vorvx{bot})
    if i==1
        openfig(strcat('R',num2str(bot),'.fig')) ;
    end
    if i==length(vorvx{bot})
        j=1;
    else
        j=i+1;
    end
    if vorvx{bot}(i,1)==vorvx{bot}(j,1)
        m=inf;
        final=[vorvx{bot}(i,1) vorvx{bot}(i,2) 90];
    else
        m=(vorvx{bot}(j,2)-vorvx{bot}(i,2))/(vorvx{bot}(j,1)-vorvx{bot}(i,1));
        final=[vorvx{bot}(i,1) vorvx{bot}(i,2) atand(m)];
    end
%     disp(atand(m));
    final=round(final,4);
    m=(final(2)-R1.initial_pos(2))/(final(1)-R1.initial_pos(1));
    o=atand(m)-R1.initial_pos(3);
    disp(o)
    d=round(pdist([R1.initial_pos(1),R1.initial_pos(2);final(1),final(2)],'euclidean'),4);
    if d~=0
        if o~=0 && i==1
            [t,flag]=inplace_rotation(o,Vr,Vl,l);
            inputs=[Vl/r Vr/r];
            if flag
                inputs(1)=-1*inputs(1);
            else
                inputs(2)=-1*inputs(2);
            end
            [time,pose]=solver(R1.model,R1.initial_pos,inputs,t);
            R1.cur_pos=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
            R1.cur_pos=round(R1.cur_pos,4);
            motion=linear_motion(R1.cur_pos,final,Vr,d/Vr);
            if motion=='r'
                temp=pose;
                %temp(:,3)=temp(:,3)*180/pi;
                [t,flag]=inplace_rotation(180,Vr,Vl,l);
                [time,pose]=solver(R1.model,R1.cur_pos,inputs,t);
                R1.cur_pos=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
                R1.cur_pos=round(R1.cur_pos,4);
            end
        else
            R1.cur_pos=R1.initial_pos;
        end  
%         if flag
%             inputs(1)=-1*inputs(1);
%         else
%             inputs(2)=-1*inputs(2);
%          end
        if i~=1
            motion=linear_motion(R1.cur_pos,final,Vr,d/Vr);
            if motion=='r'
                temp=pose;
                %temp(:,3)=temp(:,3)*180/pi;
                [t,flag]=inplace_rotation(180,Vr,Vl,l);
                [time,pose]=solver(R1.model,R1.cur_pos,inputs,t);
                R1.cur_pos=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
                R1.cur_pos=round(R1.cur_pos,4);
            end
        end
        t=d/Vr;
        inputs=[Vl/r Vr/r];
        [time,pose]=solver(R1.model,R1.cur_pos,inputs,t);
        R1.cur_pos=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
        R1.cur_pos=round(R1.cur_pos,4);
        len=length(pose);
        hold on 
        if i==1
            pause(5);
            plot(pose(len,1),pose(len,2),'b*');
            text(pose(len,1),pose(len,2),'Starting Position');
        else
            for j=1:2:len
                plot(pose(j,1),pose(j,2),'-bx');
                xlabel('X-axis');
                ylabel('Y-axis');
                %plot(R1.initial_pos(1),R1.initial_pos(2),'r.','MarkerSize',10);
                %axis([-5 5 -5 5]);
                grid on;
                pause(1);
                %hold on;   
            end
         end
        R1.cur_pos=round(R1.cur_pos,4);
    else
        R1.cur_pos=R1.initial_pos;
    end
    b=final(3)-R1.cur_pos(3);
    if b~=0
        [t,flag]=inplace_rotation(b,Vr,Vl,l);
        inputs=[Vl/r Vr/r];
        if flag
            inputs(1)=-1*inputs(1);
        else
            inputs(2)=-1*inputs(2);
        end
        [time,pose]=solver(R1.model,R1.cur_pos,inputs,t);
        R1.cur_pos=[pose(end,1) pose(end,2) pose(end,3)*180/pi];
        R1.cur_pos=round(R1.cur_pos,4);
    end
    R1.intial_pos=[];
    R1.initial_pos=R1.cur_pos;
    disp(R1.initial_pos)
    R1.cur_pos=[];
end