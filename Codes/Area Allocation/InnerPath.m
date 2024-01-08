% clear all;
% plot(vorvx{1}(:,1),vorvx{1}(:,2));
% hold on;
% for k=1:5
%     res = [];
%     for i=1:length(vorvx{1})
%         for j=1:2
%             temp = -1;
%             if vorvx{1}(i,j)<=0
%                 temp = temp * -1;
%             end
%             res(i,j) = vorvx{1}(i,j) +temp;
%         end
%     end
%     plot(res(:,1),res(:,2));
%     disp(res)
%     hold on;
%     vorvx{1} = res;
% end
% hold off;

% a =[ 5 0 ];
% b =[ 8 5];
% m = (a(2)-b(2))/(a(1)-b(1));
% d = norm(b-a);
% delta = 0.2;
% d = d- delta;
% x = 5.2 + (d * sqrt(1/(1+m*m)));
% y = 0 + m* d * sqrt(1/(1+m*m));

points = vorvx{1}(1:5,:);
i = 0;
a = [];
temp = points;
loop = 1;
beg_slope = (points(end,2)-points(1,2))/(points(end,1)-points(1,1));
while i<20
    if isempty(a)
        dist = norm(points(end,:)-points(1,:));
        delta = 0.2;
        if loop>1
            delta = delta + 0.2;
        end
        if(points(end,1)>points(1,1))
            flag = -1;
        else
            flag = 1;
        end
        x = points(end,1) +(flag * (dist-delta) * sqrt(1/(1+beg_slope*beg_slope)));
%         if(points(end,2)>points(1,2))
%             flag = -1;
%         else
%             flag = 1;
%         end
        y = points(end,2) +( beg_slope * (dist-delta) * sqrt(1/(1+beg_slope*beg_slope)));
        a = [a ;[x y]];
        i = i +1;
    else
        l = size(a,1);
        m = (points(l+1,2)-points(l,2))/(points(l+1,1)-points(l,1));
        dist = norm(points(l+1,:)-points(l,:));
        delta = 0.4;
        if loop>1
            delta = delta + 0.4;
        end
        if l==2
            delta = delta + 0.2;
        end
        if(points(l+1,1)>points(l,1))
            flag = 1;
        else
            flag = -1;
        end
        x = a(l,1) +(flag * (dist-delta) * sqrt(1/(1+m*m)));
%         if(points(l+1,2)>points(l,2))
%             flag = 1;
%         else
%             flag = -1;
%         end

        y = a(l,2) +(m * (dist-delta) * sqrt(1/(1+m*m)));
        a = [a ;[x y]];
        i = i + 1;
    end 
    if(length(a)==length(points))
        temp =[temp;a];
        points = a;
        a = [];
        loop = loop +1;
    end
end
plot(temp(:,1),temp(:,2));


