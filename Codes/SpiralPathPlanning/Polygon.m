clc;close all;clear all;

numberOfSides=inputdlg('Enter number of Sides','Polygon',[1 50]);
numberOfSides=str2num(numberOfSides{1});

if(numberOfSides)>=3
    points = createPathPoints(numberOfSides);
    DDR(points,numberOfSides);
else
    errordlg('Number of Sides must be atleast 3','Invalid Input');
end
    

