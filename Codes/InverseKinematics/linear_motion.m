function motion=linear_motion(current,final,V,t)
    x=round(current(1)+V*t*cosd(current(3)),4);
    y=round(current(2)+V*t*sind(current(3)),4);
    if x==final(1) && y==final(2)
        motion='f';
    else
        motion='r';
    end
end
