clc;
clear;
close all;

% Map settings
map_size = 10;
grid_res = 0.25;

x_grid = 0:grid_res:map_size;
y_grid = 0:grid_res:map_size;

nx = length(x_grid);
ny = length(y_grid);

% Start and goal positions
start_pos = [0.5 0.5];
goal_pos  = [9.0 9.0];

% Obstacles: [x, y, radius]
obstacles = [
    3.0  3.0  0.8;
    5.0  5.0  1.0;
    6.5  2.5  0.7;
    3.5  7.0  0.8
];

% Safety margin
robot_radius = 0.25;
safety_margin = 0.20;

% Occupancy grid
map = zeros(ny, nx);

for iy = 1:ny
    for ix = 1:nx

        px = x_grid(ix);
        py = y_grid(iy);

        for i = 1:size(obstacles,1)

            ox = obstacles(i,1);
            oy = obstacles(i,2);
            r  = obstacles(i,3) + robot_radius + safety_margin;

            if sqrt((px - ox)^2 + (py - oy)^2) <= r
                map(iy, ix) = 1;
            end
        end
    end
end

% Convert world positions to grid indices
start_idx = worldToGrid(start_pos, x_grid, y_grid);
goal_idx  = worldToGrid(goal_pos,  x_grid, y_grid);

% Run A* planner
path_idx = astarPlanner(map, start_idx, goal_idx);

if isempty(path_idx)
    error('No path found. Try changing obstacle positions.');
end

% Convert grid path to world coordinates
path = zeros(size(path_idx,1), 2);

for i = 1:size(path_idx,1)
    path(i,1) = x_grid(path_idx(i,2));
    path(i,2) = y_grid(path_idx(i,1));
end

% Reduce A* path to safe waypoints
waypoints = simplifyPath(path, 8);

% Initial robot state
x = start_pos(1);
y = start_pos(2);
theta = 0;

% Simulation settings
dt = 0.05;
T = 80;
N = T/dt;

% Controller gains
Kv = 1.0;
Kw = 3.0;

% Velocity limits
v_max = 1.0;
w_max = 3.0;

% Waypoint settings
goal_tolerance = 0.18;
current_wp = 2;

% Data storage
x_hist = zeros(1, N);
y_hist = zeros(1, N);
theta_hist = zeros(1, N);
error_hist = zeros(1, N);

step = 1;

% Robot waypoint-following simulation
for k = 1:N

    if current_wp > size(waypoints,1)
        break;
    end

    % Current target waypoint
    x_goal = waypoints(current_wp,1);
    y_goal = waypoints(current_wp,2);

    % Position error
    dx = x_goal - x;
    dy = y_goal - y;
    distance = sqrt(dx^2 + dy^2);

    % Save current data
    x_hist(step) = x;
    y_hist(step) = y;
    theta_hist(step) = theta;
    error_hist(step) = distance;
    step = step + 1;

    % Switch to next waypoint
    if distance < goal_tolerance
        current_wp = current_wp + 1;
        continue;
    end

    % Heading control
    theta_goal = atan2(dy, dx);

    error_theta = theta_goal - theta;
    error_theta = atan2(sin(error_theta), cos(error_theta));

    v = Kv * distance;
    w = Kw * error_theta;

    % Apply velocity limits
    v = min(v, v_max);
    w = max(min(w, w_max), -w_max);

    % Differential drive kinematics
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + w*dt;
end

% Trim unused data
x_hist = x_hist(1:step-1);
y_hist = y_hist(1:step-1);
theta_hist = theta_hist(1:step-1);
error_hist = error_hist(1:step-1);

% 3D animation
figure;

for k = 1:5:length(x_hist)

    clf;

    % Ground plane
    [Xg, Yg] = meshgrid(0:1:10, 0:1:10);
    Zg = zeros(size(Xg));
    mesh(Xg, Yg, Zg);
    hold on;

    % Obstacles
    for i = 1:size(obstacles,1)
        drawCylinderObstacle(obstacles(i,1), obstacles(i,2), obstacles(i,3));
    end

    % Raw A* path
    plot3(path(:,1), path(:,2), zeros(size(path,1),1), ...
          'k:', 'LineWidth', 1.5);

    % Selected waypoints
    plot3(waypoints(:,1), waypoints(:,2), ...
          zeros(size(waypoints,1),1), ...
          'r--o', 'LineWidth', 2);

    % Robot trajectory
    plot3(x_hist(1:k), y_hist(1:k), zeros(1,k), ...
          'b', 'LineWidth', 2);

    % Robot model
    drawRobot3D(x_hist(k), y_hist(k), theta_hist(k));

    grid on;
    axis equal;

    xlim([0 10]);
    ylim([0 10]);
    zlim([0 1.5]);

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

    title('3D Mobile Robot Navigation with A* Path Planning');

    view(45,25);
    drawnow;
end

% Tracking error plot
figure;

time = 0:dt:dt*(length(error_hist)-1);

plot(time, error_hist, 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('Distance to Current Waypoint [m]');
title('A* Waypoint Tracking Error');


function idx = worldToGrid(pos, x_grid, y_grid)

% Convert world position to nearest grid index
[~, ix] = min(abs(x_grid - pos(1)));
[~, iy] = min(abs(y_grid - pos(2)));

idx = [iy ix];

end


function path = astarPlanner(map, start_idx, goal_idx)

% A* search on an 8-connected grid
[ny, nx] = size(map);

open_set = start_idx;
came_from = zeros(ny, nx, 2);

g_score = inf(ny, nx);
f_score = inf(ny, nx);

g_score(start_idx(1), start_idx(2)) = 0;
f_score(start_idx(1), start_idx(2)) = heuristic(start_idx, goal_idx);

visited = false(ny, nx);

moves = [
    -1  0;
     1  0;
     0 -1;
     0  1;
    -1 -1;
    -1  1;
     1 -1;
     1  1
];

while ~isempty(open_set)

    best_index = 1;
    best_score = f_score(open_set(1,1), open_set(1,2));

    for i = 2:size(open_set,1)
        score = f_score(open_set(i,1), open_set(i,2));

        if score < best_score
            best_score = score;
            best_index = i;
        end
    end

    current = open_set(best_index,:);

    if isequal(current, goal_idx)
        path = reconstructPath(came_from, current);
        return;
    end

    open_set(best_index,:) = [];
    visited(current(1), current(2)) = true;

    for i = 1:size(moves,1)

        neighbor = current + moves(i,:);

        if neighbor(1) < 1 || neighbor(1) > ny || ...
           neighbor(2) < 1 || neighbor(2) > nx
            continue;
        end

        if map(neighbor(1), neighbor(2)) == 1 || ...
           visited(neighbor(1), neighbor(2))
            continue;
        end

        move_cost = norm(moves(i,:));
        tentative_g = g_score(current(1), current(2)) + move_cost;

        if tentative_g < g_score(neighbor(1), neighbor(2))

            came_from(neighbor(1), neighbor(2), :) = current;

            g_score(neighbor(1), neighbor(2)) = tentative_g;
            f_score(neighbor(1), neighbor(2)) = ...
                tentative_g + heuristic(neighbor, goal_idx);

            if ~isInOpenSet(open_set, neighbor)
                open_set = [open_set; neighbor];
            end
        end
    end
end

path = [];

end


function h = heuristic(a, b)

% Euclidean heuristic
h = norm(a - b);

end


function found = isInOpenSet(open_set, node)

% Check if node is already in the open set
found = false;

for i = 1:size(open_set,1)
    if isequal(open_set(i,:), node)
        found = true;
        return;
    end
end

end


function path = reconstructPath(came_from, current)

% Reconstruct path from goal to start
path = current;

while true

    previous = squeeze(came_from(current(1), current(2), :))';

    if all(previous == 0)
        break;
    end

    path = [previous; path];
    current = previous;
end

end


function waypoints = simplifyPath(path, step_size)

% Keep every N-th path point as waypoint
waypoints = path(1:step_size:end,:);

if ~isequal(waypoints(end,:), path(end,:))
    waypoints = [waypoints; path(end,:)];
end

end


function drawCylinderObstacle(xc, yc, r)

% Cylindrical obstacle
h = 0.8;

[X, Y, Z] = cylinder(r, 30);

X = X + xc;
Y = Y + yc;
Z = Z * h;

surf(X, Y, Z, 'FaceAlpha', 0.7);

end