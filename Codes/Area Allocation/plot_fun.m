function plot_fun(pos,vorvx,bnd_pnts)
    for i=1:length(pos)
        figure;
        plot(pos(i,1),pos(i,2),'r*');
        grid on;
        hold on;
        plot(vorvx{i}(:,1),vorvx{i}(:,2),'LineWidth',3);
        plot(bnd_pnts(:,1),bnd_pnts(:,2),'LineWidth',3);
        text(pos(i,1),pos(i,2),strcat('R',num2str(i)));
        hold off;
    end
    
end
