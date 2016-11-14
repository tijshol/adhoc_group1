function sendPacket( pack, src, des )
%SENDPACKET sends packet PACK from SRC to DES
    global link;
    global nextNode;
    global nextLink;
    
    disp('    ')
    disp('CALL TO SENDPACKET')
    disp(['des = ' num2str(des)])
    
    for i = 1:length(des)
        if link(src,des(i)).valid
            disp(['sent to ' num2str(des(i))])
            nextNode(des(i)).recvBuf = [nextNode(des(i)).recvBuf pack];
            nextNode(des(i)).received = true;
            nextLink(src,des(i)).state = 'active';
            nextLink(des(i),src).state = 'active';
        else
            error(['Error: Node ' num2str(src) ' and node ' num2str(des(i)) ' are not connected.']);
        end
    end
end