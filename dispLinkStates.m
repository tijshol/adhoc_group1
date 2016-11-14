function dispLinkStates()
%DISPLINKSTATES display all of the link states
%   in a table instead of all over the place
    global link;
    
    dispCell = cell(5);
    
    for i = 1:size(link,1)
        for j = 1:size(link,2)
            dispCell(i,j) = {link(i,j).state};
        end
    end
    
    disp(dispCell);
end