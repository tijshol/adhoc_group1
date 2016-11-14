%% INITIATE

global nodespace;
global DSR_src;
global DSR_des;

N = 50;             % Amount of nodes
r = 0.5;              % Maximum range
sigma = 2;          % Deviation of range
nodespace = 5;     % Size of node space

% Set node positions
nPos = nodespace*rand(N,2); % Randomize the nodes within the node space

% nodes = [0.9*nodespace*cos(2*pi*(1:N)'/N) 0.9*nodespace*sin(2*pi*(1:N)'/N)];

% Determine links
A = false(N,N);     % Adjacency matrix
weight = zeros(N);  % Weight matrix

% Calculate the adjacency and weight matrices
for i = 1:N
    for j = 1:i
        if i == j
            continue;
        end
        range = r + random('norm',0,sigma);
        dist = (nPos(i,1)-nPos(j,1))^2 + (nPos(i,2)-nPos(j,2))^2;
        if dist < range
            A(i,j) = true;
            A(j,i) = true;
            weight(i,j) = dist^2;
            weight(j,i) = weight(i,j);
        end
    end
end

% DEBUG
% Alternative graph

% N = 5;
% nPos = [0.1 0.5
%         0.3 0.5
%         0.5 0.5
%         0.7 0.5
%         0.9 0.5];
% A = logical([0 1 0 0 0
%              1 0 1 0 0
%              0 1 0 1 0
%              0 0 1 0 1
%              0 0 0 1 0]);
% weight = ones(5);
% nodespace = 1;

% Determine the degrees (sum of adjacency matrix)
d = sum(A)';

if any(find(d == 0))
    zeroDegreeN = length(find(d == 0));
    if zeroDegreeN == 1
        zeroDegreeMsg = ['There is 1 node with degree 0! Continue?'];
    else
        zeroDegreeMsg = ['There are ' num2str(zeroDegreeN) ' nodes with degree 0! Continue?'];
    end
    if strcmp(questdlg(zeroDegreeMsg), 'No')
        return;
    end
end

% Set initial node states
nodeState = cell(N,1);
nodeState(:) = {'idle'};
% nodeState(1) = {'flood'};

% Set initial link states
linkState = cell(N);
linkState(A) = {'idle'};

% Define clear receive buffer
clearBuffer = struct([]);

% Create node and link struct arrays
global node;
node = struct('state',nodeState, ...        % State of node (string)
    'pos',mat2cell(nPos,ones(N,1),2), ...   % Position of node (1x2 number)
    'degree',num2cell(d), ...               % Degree of node (number)
    'recvBuf',clearBuffer, ...              % Receive buffer (Nx1 struct)
    'memory',struct());                     % Memory of the node (struct)

global link;
link = struct('state',linkState, ...        % State of the link (string)
    'valid',num2cell(A), ...                % Adjacency (boolean)
    'weight',num2cell(weight));             % Weight (number)

global nextNode;
global nextLink;

[~, DSR_src] = min(nPos(:,1).*nPos(:,2));
[~, DSR_des] = max(nPos(:,1).*nPos(:,2));

%% PERFORM DSR

nextNode = node;
nextLink = link;

DSR_pack = struct('head', 'DSR_RREQ', ...
                  'body', struct('path', DSR_src, ...
                                 'pathWeight', 0, ...
                                 'dest', DSR_des));
broadcast(DSR_pack, DSR_src);
nextNode(DSR_src).state = 'flooded';

node = nextNode;
link = nextLink;

plotgraph;
title('Iteration 0')

exitSim = false;
titleEvent = ' searching with RREQ...';

for n = 1:20
    disp('Ready!')
    pause(0.1);
    disp(['-- ITERATION ' num2str(n) ' --'])
    
    % Go by every node
    for i = 1:N
        packetAmount = length(node(i).recvBuf);
        % Only trigger an action when the node has received a packet
        % Then handle all of the received packets
        for k = 1:packetAmount
            if k == 1
                disp(['NODE ' num2str(i) ' received packets']);
            end
            if packetAmount > 1
                disp(['(' num2str(k) '/' num2str(length(node(i).recvBuf)) '):']);
            end
            recvPacket = node(i).recvBuf(k);
            disp(recvPacket);

            switch recvPacket.head
                case 'DSR_RREQ' % Dynamic source routing request
                    % -- Packet body:
                    % dest: destination
                    % path: path already taken
                    % pathWeight: weight of path already taken
                    
                    % Determine last node in path
                    lastNode = recvPacket.body.path(end);
                        
                    % Append RSSI to the weight and add ID to the route
                    pathWeight = recvPacket.body.pathWeight + link(lastNode,i).weight;
                    path = [recvPacket.body.path; i];
                    
                    if i == recvPacket.body.dest % We found the node!
                        % Create a response packet and send it back.

                        RREPBody = struct('path', path, ...
                                          'pathWeight', pathWeight, ...
                                          'repStep', length(path)-1);

                        newPacket = struct('head','DSR_RREP', ...
                                            'body', RREPBody);
                        
                        % Append packet to array of DSR packets received
                        if ~any(strcmp('DSRpaths',fieldnames(nextNode(i).memory)))
                            nextNode(i).memory.DSRpaths = struct([]);
                        end
                        nextNode(i).memory.DSRpaths = [nextNode(i).memory.DSRpaths newPacket];
                        
                        if ~strcmp(node(i).state,'flooded')
                            sendPacket(newPacket,i,path(end-1));
                            nextNode(i).state = 'flooded';
                            titleEvent = ' replying with RREP...';
                        end
                    end
                    
                    % If not the destination and not flooded yet, broadcast the new packet                        
                    if i ~= recvPacket.body.dest && ~strcmp(node(i).state,'flooded')
                        newPacket = recvPacket;
                        newPacket.body.pathWeight = pathWeight;
                        newPacket.body.path = path;
                        broadcast(newPacket,i);
                        % Make sure after this iteration the node stops broadcasting
                        nextNode(i).state = 'flooded';
                    end
                    
                case 'DSR_RREP' % Dynamic source routing response
                    % -- Packet body:
                    % path: path going back to the source
                    % pathWeight: weight of path already taken
                    % repStep: the current step in the reply path
                    nextRepStep = recvPacket.body.repStep - 1;
                    
                    if nextRepStep == 0 % We reached the source again!
                        disp('A route is known!');
                        highlight(recvPacket.body.path);
                        exitSim = true;
                    else
                        newPacket = recvPacket;
                        newPacket.body.repStep = nextRepStep;
                        sendPacket(newPacket,i,recvPacket.body.path(nextRepStep));
                    end
            end
            if exitSim
                break;
            end
        end
        if exitSim
            break;
        end
        if packetAmount>0
            % Clear the receive buffer
            nextNode(i).recvBuf = clearBuffer;
        end
    end
    if exitSim
        title(['Route from node ' num2str(DSR_src) ' to ' num2str(DSR_des)]);
        break;
    end
    
    % Update complete node and link struct arrays

    node = nextNode;
    link = nextLink;
    
    plotgraph;
    title(['Iteration ' num2str(n) titleEvent]);
    resetLinkStates();
    
    % test test test
end