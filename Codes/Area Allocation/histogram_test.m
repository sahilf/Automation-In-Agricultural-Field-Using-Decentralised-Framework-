data = csvread('rnorm.csv',1,0);
result = zeros(length(data));
temp = zeros(1,50);
l = length(data);
for i=1:l
    for j=1:l
        angle = data(i,:) * transpose(data(j,:));
        if angle>cosd(20) && angle<cosd(0)
            result(i,j) = angle;
        end
    end
%     figure;
    temp(i) = histcounts(result(i));
end
% disp(result)
% for i=1:l
%     figure;
%     
% end



