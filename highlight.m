function highlight( route )
%HIGHLIGHT highlight a route in the graph.
    global node;
    global link;
    
    col = [0.6 1 0.2];
    hold on;
    for i = 1:length(route)-1
        plot([node(route(i)).pos(1) node(route(i+1)).pos(1)], ...
             [node(route(i)).pos(2) node(route(i+1)).pos(2)], ...
             'Color',col,'LineWidth',4-3*(link(route(i),route(i+1)).weight/max([link.weight])));
    end
    for i = 1:length(route)
        plot(node(route(i)).pos(1),node(route(i)).pos(2),'.','Color',col,'MarkerSize',40);
    end
end

