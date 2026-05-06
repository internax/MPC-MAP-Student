function [path] = astar(read_only_vars, public_vars)
%ASTAR Finds shortest path using A* on occupancy grid

occ_map = read_only_vars.discrete_map.map;
step    = read_only_vars.map.discretization_step;
limits  = read_only_vars.map.limits;

% Tvrda dilatace: 1 bunka (0.2m) - fyzicka bezpecnostni zona
hard_clearance = round(0.3 / step);
occ_map        = inflate_obstacles(occ_map, hard_clearance);

% Soft cost mapa- vzdalenost od prekazek 
dist_map    = dist_transform(occ_map);
penalty_map = 4.0 * exp(-dist_map / 1.5);

[nrows, ncols] = size(occ_map);

% Prevod spojitych souradnic na diskretni
start_cont = public_vars.estimated_pose(1:2);
goal_cont  = read_only_vars.map.goal(1:2);

start = cont2disc(start_cont, limits, step, nrows, ncols);
goal  = cont2disc(goal_cont,  limits, step, nrows, ncols);

% A* datove struktury
g = inf(nrows, ncols);
g(start(1), start(2)) = 0;
came_from = zeros(nrows, ncols, 2);
closed    = false(nrows, ncols);

h = @(r,c) sqrt((r - goal(1))^2 + (c - goal(2))^2); % anon fce

% open list
open_list = [h(start(1), start(2)), start(1), start(2)];

% 8-smrove pohyby 
moves = [-1,-1,sqrt(2); -1,0,1; -1,1,sqrt(2);
          0,-1,1;                 0,1,1;
          1,-1,sqrt(2);  1,0,1;  1,1,sqrt(2)];

while ~isempty(open_list)
    [~, idx] = min(open_list(:,1));
    r = open_list(idx, 2);
    c = open_list(idx, 3);
    open_list(idx, :) = [];

    if closed(r, c), continue; end
    closed(r, c) = true;

    % Cil nalezen
    if r == goal(1) && c == goal(2)
        path = reconstruct_path(came_from, start, goal, limits, step);
        return;
    end

    % Rozvin sousedy
    for i = 1:8
        nr = r + moves(i,1);
        nc = c + moves(i,2);

        if nr<1 || nr>nrows || nc<1 || nc>ncols, continue; end
        if occ_map(nr,nc) || closed(nr,nc),       continue; end

        new_g = g(r,c) + moves(i,3) + penalty_map(nr,nc);
        if new_g < g(nr,nc)
            g(nr,nc)            = new_g;
            came_from(nr,nc,:)  = [r, c];
            f = new_g + h(nr, nc);
            open_list = [open_list; f, nr, nc];
        end
    end
end

path = [];
disp('A*: cesta nenalezena');

end

% pomocne funkce aby kod fungoval bez toolboxu

function disc = cont2disc(cont, limits, step, nrows, ncols)
    col  = round((cont(1) - limits(1)) / step) + 1;
    row  = round((cont(2) - limits(2)) / step) + 1;
    col  = max(1, min(ncols, col));
    row  = max(1, min(nrows, row));
    disc = [row, col];
end

function cont = disc2cont(disc, limits, step)
    x    = (disc(2) - 1) * step + limits(1);
    y    = (disc(1) - 1) * step + limits(2);
    cont = [x, y];
end

function path = reconstruct_path(came_from, start, goal, limits, step)
    path = [];
    cur  = goal;
    while ~(cur(1) == start(1) && cur(2) == start(2))
        path = [disc2cont(cur, limits, step); path];
        cur  = squeeze(came_from(cur(1), cur(2), :))';
    end
    path = [disc2cont(start, limits, step); path];
end

% nahrada bwdist pro výpočet vzdálenosti pro kazdou bnku
function dist_map = dist_transform(occ_map)
    [nrows, ncols] = size(occ_map);
    dist_map = inf(nrows, ncols);

    moves = [-1,-1,sqrt(2); -1,0,1; -1,1,sqrt(2);
              0,-1,1;                0,1,1;
              1,-1,sqrt(2);  1,0,1;  1,1,sqrt(2)];

    % init - prekazky maji vzdalenost 0
    dist_map(occ_map) = 0;
    [qr, qc] = find(occ_map);
    queue = [qr, qc];

    while ~isempty(queue)
        r = queue(1,1);
        c = queue(1,2);
        queue(1,:) = [];

        for i = 1:8
            nr = r + moves(i,1);
            nc = c + moves(i,2);
            if nr<1||nr>nrows||nc<1||nc>ncols, continue; end
            new_d = dist_map(r,c) + moves(i,3);
            if new_d < dist_map(nr,nc)
                dist_map(nr,nc) = new_d;
                queue = [queue; nr, nc];
            end
        end
    end
end

