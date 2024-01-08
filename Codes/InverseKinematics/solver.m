function [t,pose]=solver(R,position,inputs,t)
position(3)=position(3)*pi/180;
[t,pose]=ode45(@(t,pose) derivative(R,pose,inputs),[0 t],position);

