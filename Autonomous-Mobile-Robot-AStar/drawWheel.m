function drawWheel(x, y, theta, xOffset, yOffset, r, width)

% Wheel position in robot frame
localPos = [xOffset; yOffset; r];

% Rotation matrix
Rz = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1
];

% Transform wheel center to global frame
globalPos = Rz * localPos;

xc = x + globalPos(1);
yc = y + globalPos(2);
zc = globalPos(3);

% Generate wheel geometry
[Xc, Yc, Zc] = cylinder(r, 20);

Zc = Zc * width - width/2;

X = Xc;
Y = Zc;
Z = Yc;

% Rotate wheel according to robot heading
points = [X(:)'; Y(:)'; Z(:)'];
points = Rz * points;

X = reshape(points(1,:), size(X)) + xc;
Y = reshape(points(2,:), size(Y)) + yc;
Z = reshape(points(3,:), size(Z)) + zc;

% Draw wheel
surf(X, Y, Z);

end