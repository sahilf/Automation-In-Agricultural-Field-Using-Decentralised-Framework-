%%Initial Settings
clear all;                                          %Clear all the variables declared so far
close all;                                          %Close all the figures
clc;                                                %Clear the console
tol = 1e-07;                                        %tolerance value used in "inhull.m" (larger value high precision, possible numerical error)
d=2;
%%
%Input Positions of the robot
prompt={"X-Coordinates:","Y-Coordinates:"};         %Prompt Fields fot input dialog box
name="Positions";                                   %Name of the input dialog box
answer=inputdlg(prompt,name,[1 50]);                %Accept input from the dialog box
pos0=[str2num(answer{1});str2num(answer{2})]';  %Position points from the given input
disp(pos0);
n=length(pos0);                                      %Number of position points
%%
%Input Boundary Points
answer=inputdlg(prompt,"Boundary Points",[1 50]);   %Acccept input for boundaries from the input dialog box
bnd0=[str2num(answer{1});str2num(answer{2})]';
disp(bnd0);
m=length(bnd0);
% bnd=[0,0;-1,0;1,0;-1,1;0,2;1,1;0,0.5];
K = convhull(bnd0);
bnd_pnts=(bnd0(K,:));
% plot(bnd(:,1),bnd(:,2),"*");
% hold on;
% plot(bnd_pnts(:,1),bnd_pnts(:,2));
%%
% inhull.m efficiently checks if points are inside a convex hull in d dimensions
%% take points that are in the boundary convex polytope
in = inhull(pos0,bnd0,[],tol);
u1 = 0;
for i = 1:size(pos0,1)
    if in(i) ==1
        u1 = u1 + 1;
        pos(u1,:) = pos0(i,:);
    end
end
%We use the function to choose points that are inside the defined boundary

%% 
% =========================================================================
% INPUTS:
% pos       points that are in the boundary      n x d matrix (n: number of points d: dimension) 
% bnd_pnts  points that defines the boundary     m x d matrix (m: number of vertices for the convex polytope
% boundary d: dimension)
% -------------------------------------------------------------------------
% OUTPUTS:
% vornb     Voronoi neighbors for each generator point:     n x 1 cells
% vorvx     Voronoi vertices for each generator point:      n x 1 cells
% =========================================================================

[vornb,vorvx] = polybnd_voronoi(pos,bnd_pnts);

%% PLOT
for i = 1:size(vorvx,2)
    col(i,:)= rand(1,3);
end
figure;
for i = 1:size(pos,1)
plot(vorvx{i}(:,1),vorvx{i}(:,2),'-r')
hold on;
end
plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
hold on;
plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
plot(bnd0(:,1),bnd0(:,2),"*");