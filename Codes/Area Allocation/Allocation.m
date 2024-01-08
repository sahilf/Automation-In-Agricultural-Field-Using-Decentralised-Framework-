
clear all;clc;close all;
tol = 1e-07;                                        
d=2;
n=inputdlg('Enter number of robots','Robots',[1 50]);
n=str2num(n{1});
s='Enter position of R ';
prompt=cell(1,n);
for i =1:n
     prompt{i}=strcat(s,num2str(i));
end
%disp(prompt)
name="Positions";                                   
answer=inputdlg(prompt,name,[1 50]); 
org_pos=[];
for i=1:n
    temp=str2num(answer{i});
    org_pos=[org_pos;temp];
end
    pos0=[org_pos(:,1) org_pos(:,2)];

m=inputdlg('Enter number of bounday points','Robots',[1 50]);
m=str2num(m{1});
s='Enter corner point ';
prompt=cell(1,m);
for i =1:m
     prompt{i}=strcat(s,num2str(i));
end
%disp(prompt)
name="Boundary";                                   
answer=inputdlg(prompt,name,[1 50]); 
bnd0=[];
for i=1:m
    temp=str2num(answer{i});
    bnd0=[bnd0;temp];
end
K = convhull(bnd0);
bnd_pnts=(bnd0(K,:));
in = inhull(pos0,bnd0,[],tol);
u1 = 0;
for i = 1:size(pos0,1)
    if in(i) == 1
        u1 = u1 + 1;
        pos(u1,:) = pos0(i,:);
    end
end

[vornb,vorvx] = polybnd_voronoi(pos,bnd_pnts);
for i=1:length(vorvx)
    vorvx{i}=round(vorvx{i},4);
end
% plot_fun(pos,vorvx,bnd_pnts);

for i = 1:size(vorvx,2)
    col(i,:)= rand(1,3);
end
temp=[];
for i=1:length(pos)
    temp=[temp;strcat('R',num2str(i))];
end
figure;
for i = 1:size(pos,1)
plot(vorvx{i}(:,1),vorvx{i}(:,2),'-r','LineWidth',2)
hold on;
end
plot(bnd_pnts(:,1),bnd_pnts(:,2),'-b','LineWidth',2);
hold on;
grid on;
plot(pos(:,1),pos(:,2),'.b');
text(pos(:,1),pos(:,2),temp);
xlabel('x-axis');
ylabel('y-axis');
title('Area Allocation');
plot(bnd0(:,1),bnd0(:,2),".");
% 
% for i=1:length(vorvx)
%     disp(strcat('Area coordinates of R',int2str(i)));
%     disp(vorvx{i});
%     
% end
% disp('Running Path Generation Script');
% run Coordinates_Generation.m
