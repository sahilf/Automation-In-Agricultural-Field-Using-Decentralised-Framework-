function updated_connection=rnetwork_check(cur_pos,pos_values,r,connection,id)
for k=1:3
    if k~=id
        d=pdist([cur_pos;pos_values(k,1:2)],'euclidean');
        if d<r
            connection(k)=1;
            plot([cur_pos(1) pos_values(k,1)],[cur_pos(2) pos_values(k,2)],'k');
            pause(1);
        else
            connection(k)=0;
        end
    end
end
updated_connection=connection;
end