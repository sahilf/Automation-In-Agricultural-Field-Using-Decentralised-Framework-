function plot_fun(m,n,pose)
    figure;
    len=length(pose);
    for i=1:1:len
        plot(m,n,'-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6])
        x=pose(i,1)+(0.001*cos(pose(i,3)));
        y=pose(i,2)+(0.001*sin(pose(i,3)));
        if i==1
            plot([pose(i,1) x],[pose(i,2) y],'-b*','MarkerSize',10);
        elseif i==len
            plot([pose(i,1) x],[pose(i,2) y],'-g.','MarkerSize',10);
        else
            plot([pose(i,1) x],[pose(i,2) y]);
        end
        grid on;
        pause(1);
        hold on;
    end
end