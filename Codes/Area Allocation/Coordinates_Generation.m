clearvars -except org_pos vorvx 
path_points = cell(1,4);
for bot=1:4
    for i=1:length(vorvx{bot})
        if org_pos(bot,1:2)==vorvx{bot}(i,1:2)
            bot_pos=i;
        end
    end
    l=length(vorvx{bot});
    if vorvx{bot}(bot_pos,1)== vorvx{bot}(bot_pos+1,1)
        i=1;
    else
        i=2;
    end
    if vorvx{bot}(bot_pos,i)>vorvx{bot}(bot_pos-1,i)
        delta=-0.2;
    else
        delta=0.2;
    end
    a=[vorvx{bot}(bot_pos,:);vorvx{bot}(bot_pos+1,:)];
    m=a(end,1);
    n=a(end,2);
    if i==1
        m=m+delta;
    else
        n=n+delta;
    end

    a=[a;[m,n]];
    while true
        n=a(end,i);
        if i==1
            m=a(end-2,2);
            a = [a;[n,m]];
            a=[a;[n+delta,m]];
        else
            m=a(end-2,1);
            a=[a;[m,n]];
            a=[a;[m,n+delta]];
            t = [m,n+delta];
        end
        a=round(a,4);
        if isequal(a(end-1,:),vorvx{bot}(bot_pos-1,:))
           a(end,:) =[];
           path_points{bot} = a;
           break
        end
    end
end
disp(path_points);



% bot=4;
% l=length(vorvx{bot});
% if vorvx{bot}(1,1)== vorvx{bot}(2,1)
%     i=1;
% else
%     i=2;
% end
% if vorvx{bot}(1,i)>vorvx{bot}(l-1,i)
%     delta=-0.2;
% else
%     delta=0.2;
% end
% a=[vorvx{bot}(1,:);vorvx{bot}(2,:)];
% m=a(end,1);
% n=a(end,2);
% if i==1
%     m=m+delta;
% else
%     n=n+delta;
% end
% 
% a=[a;[m,n]];
% while 1
%     n=a(end,i);
%     if i==1
%         m=a(end-2,2);
%         a=[a;[n,m]];
%         a=[a;[n+delta,m]];
%     else
%         m=a(end-2,1);
%         a=[a;[m,n]];
%         a=[a;[m,n+delta]];
%     end
%     a=round(a,4);
%     if a(length(a),1)==vorvx{bot}(l-1,1) && a(length(a),2)==vorvx{bot}(l-1,2)
%        disp('hey');
%        break
%     end
% end








