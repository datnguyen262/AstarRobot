clear;
clc;

 map_size = [46, 42];
    % obstacle
    obs0 = [1:map_size(1), ... 
        map_size(1):map_size(1):map_size(1) * map_size(2), ...
        1:map_size(1):(map_size(1) - 1) * map_size(2), ...
        map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];

obs1 = [577:1:581,...%
        581:46:995,...%
        577:46:991,...%
        992:1:994,...%
        624:1:626,...
        670:1:672,...
        716:1:718,...
        762:1:764,...
        808:1:810,...
        854:1:856,...
        900:1:902,...
        946:1:948
        ];
    
obs2 = [194:1:206,...%
        206:46:483,...%
        240:46:470,...%
        471:1:481,...%
        241:1:251,...
        287:1:297,...
        333:1:343,...
        379:1:389,...
        425:1:435];

obs3 = [171:1:177,...%
        223:46:546,...%
        539:1:545,...%
        217,263,...%
        264:1:267,...%
        313:46:451,...%
        447:1:450,...%
        493,...%
        222:46:498,...
        218:1:221,...
        494:1:497];

obs4 = [1337:1:1349,...%
        1395:46:1809,...%
        1797:1:1808,...%
        1383:46:1797,...%
        1384:1:1394,...
        1430:1:1440,...
        1476:1:1486,...
        1522:1:1532,...
        1568:1:1578,...
        1614:1:1624,...
        1660:1:1670,...
        1706:1:1716,...
        1752:1:1762];
obs5 = [1322:1:1325,...%
        1368:46:1506,...%
        1371:46:1693,...%
        1683:1:1692,...%
        1499:1:1505,...%
        1545:46:1637,...%
        1369,1370,...
        1415,1416,...
        1461,1462,...
        1507,1508,...
        1546:1:1554,...
        1592:1:1600,...
        1638:1:1646];
        
obstacle = [obs0, obs1,obs2,obs3,obs4,obs5];
    % creat grid
    clf;
    hold on;
    grid_map = generate_grid(map_size, obstacle);

    % save and plot
    % save gridmap_46x42_scene1 grid_map
    start = [7,5];
    goal = [40,15];
    plot_grid(grid_map);
    type = 1;
    [path, path_pos] = A_star(grid_map, start, goal, type);
    grid_with_path = grid_map + path*3;
    
    plot_grid(grid_with_path);
    

function grid_map = generate_grid(size, obstacle)
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end

function plot_grid(grid_map)
    cmap = [1 1 1; ...  
            0 0 0;...
            1 0 0;];
    colormap(cmap);
    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on
    set(gca,'xtick', 1:cols, 'ytick', 1:rows);
    axis image;
    
    for row = 1:rows
        line([1, cols + 1], [row, row], 'Color','b');
    end
    for col = 1:cols
        line([col, col], [1, rows + 1], 'Color','b');
    end
end


function [path, path_pos] = A_star(gridmap, start, goal, type)

% Set up the A* algorithm
[N, M] = size(gridmap); % get size of the gridmap
closed_list = zeros(N,M);   % passed nodes
open_list = zeros(N,M);     % to-go nodes (queue)
back_track = zeros(N,M,2);
g_score = inf(N,M); % set g_score & f_score to infinity to indicate unexplore space
f_score = inf(N,M);
g_score(start(1), start(2)) = 0;    % set g_score at the start to be 0
f_score(start(1), start(2)) = heuristic(start, goal, gridmap, type);   % set f_score equal to the initial heuristic

% Start the search
open_list(start(1), start(2)) = 1;
while sum(open_list(:)) > 0
    current = find_open_list_node_with_min_f_score(open_list, f_score);
    if isequal(current, goal)
        [path, path_pos] = reconstruct_path(back_track, goal);
        return; % end the algorithm
    end
    open_list(current(1), current(2)) = 0;  % pop the position from open_list
    closed_list(current(1), current(2)) = 1    % and add to closed_list
    neighbors = get_neighbors(current, gridmap);    % get adjacent positions and 
    for i = 1:length(neighbors)
        neighbor = neighbors(i,:);
        if closed_list(neighbor(1), neighbor(2)) == 1
            continue;   % do not care for visited nodes
        end
        tentative_g_score = g_score(current(1), current(2)) + cost_function(current, neighbor); % current cost + cost to move to neighbor nodes
        if open_list(neighbor(1), neighbor(2)) == 0 || tentative_g_score < g_score(neighbor(1), neighbor(2))    % check if this neighbor is not in queue or fit more effeciently in this path 
            back_track(neighbor(1), neighbor(2), :) = current;  % assign current position as the previous step of this neighbor
            g_score(neighbor(1), neighbor(2)) = tentative_g_score;  % update g_score & f_score
            f_score(neighbor(1), neighbor(2)) = tentative_g_score + heuristic(neighbor, goal, gridmap, type);
            open_list(neighbor(1), neighbor(2)) = 1; % add this node to queue list end loop again
            
        end
    end
end
return;
end

function cost = cost_function(current, neighbor)
    dx = abs(current(1) - neighbor(1));
    dy = abs(current(2) - neighbor(2));
    if dx + dy == 2
        cost = 1.414; % diagonal cost
    else
        cost = 1; % straight cost
    end
end

function neighbors = get_neighbors(current, gridmap)
    [N, M] = size(gridmap);
    neighbors = zeros(8,2);
    count = 0;
    for i = -1:1
        for j = -1:1
            if i == 0 && j == 0
                continue;   % do not count the current node
            end
            if current(1)+i < 1 || current(1)+i > N || current(2)+j < 1 || current(2)+j > M
                continue;   % do not go over boundaries
            end
            if gridmap(current(1)+i, current(2)+j) == 2
                continue;   % do not count if there is obstacle
            end
            count = count + 1;
            neighbors(count,:) = [current(1)+i, current(2)+j];
        end
    end
    neighbors = neighbors(1:count,:);  % only keep non-zero elements
end



function h = heuristic(node, goal, gridmap, type)
    if gridmap(node(1), node(2)) == 2
        h = Inf;
    else
        if type==0
            h = sqrt((node(1)-goal(1))^2 + (node(2)-goal(2))^2);
        end
        if type==1 %mahattan
            h = abs(node(1)-goal(1)) + abs(node(2)-goal(2));
        end
    end
end

function node = find_open_list_node_with_min_f_score(open_list, f_score)
    [min_f_score, ~] = min(f_score(open_list == 1));
    [i, j] = ind2sub(size(f_score), find(f_score == min_f_score & open_list == 1));
    node = [i(1), j(1)];
end

function [path, path_pos] = reconstruct_path(back_track, base_current)
    [N, M, ~] = size(back_track);
    path = zeros(N, M);

    current = base_current;
    while current(1) > 0 || current(2) > 0
        path(current(1), current(2)) = 1; % set current position to 1 in path matrix
        current = back_track(current(1), current(2), :);
        current = reshape(current, 1, []);
    end

    [row, ~] = find(path==1);
    [len, ~] = size(row);
    path_pos = zeros(len, 2);
    
    current = base_current;
    for i=1:len
        if i==len
            path_pos(len, :)= reshape(base_current, 1, []);
            break
        end
    
        current = back_track(current(1), current(2), :);
   
        path_pos(len-i,:) = current;
    end
end

