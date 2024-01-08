function [t,flag]=inplace_rotation(angle,Vr,Vl,l)
    if angle>0
        Vl=-1*Vl;
        flag=1;
    else
        Vr=-1*Vr;
        flag=0;
    end
    w=(Vr-Vl)/l;
    t=(angle*pi/180)/w;
end