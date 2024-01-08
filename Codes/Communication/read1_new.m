function [a,condn]=read1_new(t,flag)
    timer=0;
    condn=1;
    temp = [];
    a = [];
    if flag
        while t.BytesAvailable == 0
                pause(1);
        end
        t.ByteOrder = 'littleEndian';
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
            start = tic;
            pause(1)
            stop = toc(start);
            timer = timer + stop;
            if timer >=15
                condn=0;
                break
            end
         end
        if condn
            t.ByteOrder = 'littleEndian';
            if t.BytesAvailable
                a = fread(t, t.BytesAvailable,'double');
            end
            pause(2);
        end
    end
end