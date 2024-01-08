function write2(t,data,flag)
    if flag
        for i =1:length(data)
            write(t, data(i,:));
            flush(t);
            pause(1);
        end
     else
        disp('Transferred Data');
        disp(data(:,1));
        write(t, data(:,1),'double');
        pause(12);
     end
end