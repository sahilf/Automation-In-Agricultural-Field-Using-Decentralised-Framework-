function read1(t,flag)
    if flag
        while t.BytesAvailable == 0
                pause(1)
        end
        t.ByteOrder = 'littleEndian';
        timer =0;
        temp = [];
        a = [];
        while 1
            start = tic;
            if t.BytesAvailable
                data = fread(t, t.BytesAvailable,'double');
                for i=1:length(data)
                    temp = [temp data(i)];
                end
            else
                stop = toc(start);
                timer = timer + stop;
                if timer >=15
                    break
                end
            end
        end
        for i=1:2:64
            a = [a; temp(1,i) temp(1,i+1)];
        end
        disp('Received Path data....');
        disp(a)
    else
         while t.BytesAvailable == 0
            pause(1)
        end
        t.ByteOrder = 'littleEndian';
        if t.BytesAvailable
            data = fread(t, t.BytesAvailable,'double');
            disp('Received Data');
            disp(data);
        end
        pause(2);
    end
end