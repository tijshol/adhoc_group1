function resetLinkStates()
%RESETLINKSTATES sets all the link and nextLink states to idle
    global link;
    global nextLink;
    
    for i = 1:size(link,1)
        for j = 1:size(link,2)
            if link(i,j).valid
                link(i,j).state = 'idle';
                nextLink(i,j).state = 'idle';
            end
        end
    end
end