function drawRobot3D(x, y, theta)

% Robot body dimensions
L = 0.7;
W = 0.45;
H = 0.20;

% Body vertices
vertices = [
    -L/2 -W/2 0;
     L/2 -W/2 0;
     L/2  W/2 0;
    -L/2  W/2 0;
    -L/2 -W/2 H;
     L/2 -W/2 H;
     L/2  W/2 H;
    -L/2  W/2 H
];

% Body faces
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

% Rotation matrix around z-axis
Rz = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1
];

% Transform body to global frame
vertices = (Rz * vertices')';
vertices(:,1) = vertices(:,1) + x;
vertices(:,2) = vertices(:,2) + y;

% Draw robot body
patch('Vertices', vertices, ...
      'Faces', faces, ...
      'FaceAlpha', 0.8);

% Heading direction
quiver3(x, y, H+0.1, ...
        0.6*cos(theta), ...
        0.6*sin(theta), ...
        0, ...
        'LineWidth', 2);

% Draw wheels
drawWheel(x, y, theta, 0,  W/2 + 0.05, 0.12, 0.08);
drawWheel(x, y, theta, 0, -W/2 - 0.05, 0.12, 0.08);

end