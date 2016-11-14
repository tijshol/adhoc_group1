function broadcast( pack, src )
%BROADCAST broadcasts packet PACK from SRC to all of its neighbors
    global link;
    des = find([link(src,:).valid]); % Find the neighbors
    sendPacket(pack, src, des);
end

