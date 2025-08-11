% Clear all variables
clear all

% Define inputs
d0 = 12.0;  % Base height (meters)
d1 = 6.0;   % Length of the first arm segment (meters)
d2 = 4.0;   % Length of the second arm segment (meters)
d3 = 2.0;   % Length of the third arm segment (meters)
P = [5; 5; 5; -pi/2];  % Target end-effector position and orientation [x; y; z; alpha]


% Define symbolic variables
syms l0 l1 l2 l3 tht1 tht2 tht3 tht4 real

% Denavit-Hartenberg transformation matrices for each joint
T01 = DHmethode(0, 0, l0, tht1);              % First joint
T12 = DHmethode(0, sym(pi/2), 0, tht2);       % Second joint
T23 = DHmethode(l1, 0, 0, tht3);              % Third joint
T34 = DHmethode(l2, 0, 0, tht4);              % Fourth joint
T45 = DHmethode(l3, 0, 0, 0);                 % End effector

% Compute the overall transformation matrix
T05 = T01 * T12 * T23 * T34 * T45;             
T05 = vpa(T05, 2);                             % Simplify and display transformation matrix

% Inverse kinematics
x = P(1);
y = P(2);
z = P(3);
alpha = P(4);

% Compute joint angles using inverse kinematics
q1 = atan2(y, x);
Ex = x / cos(q1);
Ey = y;
Ez = z - d0;
Ex = Ex - d3 * cos(alpha);
Ez = Ez - d3 * sin(alpha);

C = ((Ex^2) + (Ez^2) - d1^2 - d2^2) / (2 * d1 * d2);
q31 = acos(C);
q32 = -acos(C);

q21 = atan2(Ez, Ex) - atan2(d2 * sin(q31), d1 + d2 * cos(q31));
q41 = alpha - q21 - q31;

q22 = atan2(Ez, Ex) - atan2(d2 * sin(q32), d1 + d2 * cos(q32));
q42 = alpha - q22 - q32;

Q1 = [q1, q21, q31, q41];
Q2 = [q1, q22, q32, q42];

% Define the lengths of the arm segments (in meters)
L0 = d0;
L1 = d1;
L2 = d2;
L3 = d3;

% Define the joint angles (in radians)
tht1 = q1;  % Base rotation
tht2 = q21;
tht3 = q31;
tht4 = q41;

% Calculate the positions of the end effector and joints
x1 = 0;
y1 = 0;
z1 = L0;

x2 = L1 * cos(tht1) * cos(tht2);
y2 = L1 * cos(tht2) * sin(tht1);
z2 = L0 + L1 * sin(tht2);

x3 = L1 * cos(tht1) * cos(tht2) - L2 * (cos(tht1) * sin(tht2) * sin(tht3) - cos(tht1) * cos(tht2) * cos(tht3));
y3 = L1 * cos(tht2) * sin(tht1) - L2 * (sin(tht1) * sin(tht2) * sin(tht3) - cos(tht2) * cos(tht3) * sin(tht1));
z3 = L0 + L2 * (cos(tht2) * sin(tht3) + cos(tht3) * sin(tht2)) + L1 * sin(tht2);

x4 = L1 * cos(tht1) * cos(tht2) - L3 * (sin(tht4) * (cos(tht1) * cos(tht2) * sin(tht3) + cos(tht1) * cos(tht3) * sin(tht2)) + cos(tht4) * (cos(tht1) * sin(tht2) * sin(tht3) - cos(tht1) * cos(tht2) * cos(tht3))) - L2 * (cos(tht1) * sin(tht2) * sin(tht3) - cos(tht1) * cos(tht2) * cos(tht3));
y4 = L1 * cos(tht2) * sin(tht1) - L3 * (sin(tht4) * (cos(tht2) * sin(tht1) * sin(tht3) + cos(tht3) * sin(tht1) * sin(tht2)) + cos(tht4) * (sin(tht1) * sin(tht2) * sin(tht3) - cos(tht2) * cos(tht3) * sin(tht1))) - L2 * (sin(tht1) * sin(tht2) * sin(tht3) - cos(tht2) * cos(tht3) * sin(tht1));
z4 = L0 + L3 * (sin(tht4) * (cos(tht2) * cos(tht3) - sin(tht2) * sin(tht3)) + cos(tht4) * (cos(tht2) * sin(tht3) + cos(tht3) * sin(tht2))) + L2 * (cos(tht2) * sin(tht3) + cos(tht3) * sin(tht2)) + L1 * sin(tht2);

% Define the coordinates of the arm
x = [0, x1, x2, x3, x4];
y = [0, y1, y2, y3, y4];
z = [0, z1, z2, z3, z4];

% Define the colors of each link
colors = ['r', 'g', 'b', 'c'];

% Plot the arm in 3D
fig = figure();
ax = axes('Parent', fig);
grid(ax, 'on');
view(3);  % Set the view to 3D
a = d1 + d2 + d3;
b = d0 + d1 + d2 + d3;
xlim([-a, a]);
ylim([-a, a]);
zlim([0, b]);
xlabel('X');
ylabel('Y');
zlabel('Z');

% Draw the links of the arm
for i = 1:length(x) - 1
    line([x(i), x(i + 1)], [y(i), y(i + 1)], [z(i), z(i + 1)], 'Color', colors(i), 'LineWidth', 2);
    hold on;
end

% Plot the joints
scatter3(ax, x, y, z, 50, 'k', 'filled');
hold on;

% Plot the point
scatter3(ax, x4, y4, 0, 'filled');
hold on;
