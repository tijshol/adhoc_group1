function plotgraph()
%PLOTNODES plot the nodes in their current state

    global node;
    global link;
    global nodespace;
    global DSR_src;
    global DSR_des;
    
    N = length(node);       
    clf;
    hold on;
    
    % Links plotting
    for i = 1:N
        for j = 1:i
            if ~link(i,j).valid
                continue
            else
                switch link(i,j).state
                    case 'flood'
                        col = [0 0.8 0];
                    case 'idle'
                        col = [0.5 0.5 1];
                    case 'active'
                        col = [1 1 0.4];
                end
                plot([node(i).pos(1) node(j).pos(1)], ...
                     [node(i).pos(2) node(j).pos(2)], ...
                     'Color',col,'LineWidth',4-3*(link(i,j).weight/max([link.weight])));
            end
        end
    end
    
    % Nodes plotting
    for i = 1:N
        if ~node(i).degree
            plot(node(i).pos(1),node(i).pos(2),'r.','MarkerSize',60);
        end
        switch node(i).state
            case 'flood'
                col = [0 1 0];
                size = 40;
            case 'idle'
                col = [0 0 1];
                size = 30;
            case 'flooded'
                col = [0 0.3 0];
                size = 30;
            otherwise
                col = [0 0 0];
                size = 30;
        end
        plot(node(i).pos(1),node(i).pos(2),'.','Color',col,'MarkerSize',size);
    end
    plot(node(DSR_src).pos(1),node(DSR_src).pos(2),'r.','MarkerSize',40);
    plot(node(DSR_des).pos(1),node(DSR_des).pos(2),'g.','MarkerSize',40);

    axis([0 nodespace 0 nodespace])
end

