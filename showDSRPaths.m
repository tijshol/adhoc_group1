function showDSRPaths()
%SHOWDSRPATHS Show the paths as obtained by the DSR algorithm
    global node;
    global DSR_des;
    Npaths = length(node(DSR_des).memory.DSRpaths);
    for i = 1:Npaths
        plotgraph;
        highlight(node(DSR_des).memory.DSRpaths(i).body.path);
        title(['Path (' num2str(i) '/' num2str(Npaths) '); weight: ' num2str(node(DSR_des).memory.DSRpaths(i).body.pathWeight)]);
        pause(0.3);
    end
    plotgraph;
end

