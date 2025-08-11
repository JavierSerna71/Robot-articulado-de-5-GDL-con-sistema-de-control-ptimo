% Limpiar workspace
clear all;
clc;
pkg load symbolic;  % Cargar paquete simbólico

% ========== CONFIGURACIÓN NUMÉRICA ==========
% Longitudes físicas (metros)
d0 = 12.0;  % Altura base
d1 = 6.0;   % Brazo 1
d2 = 4.0;   % Brazo 2
d3 = 2.0;   % Brazo 3

% Posición/orientación deseada [x; y; z; alpha] (alpha en radianes)
P = [5; 5; 5; -pi/2];

% ========== CINEMÁTICA INVERSA (NUMÉRICA) ==========
% 1. Calcular theta1 (rotación base)
q1 = atan2(P(2), P(1));

% 2. Calcular theta2, theta3, theta4
Ex = P(1) / cos(q1);
Ey = P(2);
Ez = P(3) - d0;

% Ajustar por orientación (alpha)
Ex = Ex - d3 * cos(P(4));
Ez = Ez - d3 * sin(P(4));

% Soluciones para theta3
C = (Ex^2 + Ez^2 - d1^2 - d2^2) / (2 * d1 * d2);
q31 = acos(C);
q32 = -acos(C);

% Soluciones para theta2
q21 = atan2(Ez, Ex) - atan2(d2 * sin(q31), d1 + d2 * cos(q31));
q22 = atan2(Ez, Ex) - atan2(d2 * sin(q32), d1 + d2 * cos(q32));

% Soluciones para theta4
q41 = P(4) - q21 - q31;
q42 = P(4) - q22 - q32;

% Dos configuraciones posibles
Q1 = [q1, q21, q31, q41];
Q2 = [q1, q22, q32, q42];

% Usamos Q1 para la visualización
tht1 = Q2(1);
tht2 = Q2(2);
tht3 = Q2(3);
tht4 = Q2(4);

% ========== CÁLCULO DE POSICIONES (NUMÉRICO) ==========
% Coordenadas de articulaciones
x0 = 0; y0 = 0; z0 = 0;
x1 = 0; y1 = 0; z1 = d0;

x2 = d1 * cos(tht1) * cos(tht2);
y2 = d1 * cos(tht2) * sin(tht1);
z2 = d0 + d1 * sin(tht2);

x3 = x2 + d2 * (cos(tht1)*cos(tht2+tht3));
y3 = y2 + d2 * (sin(tht1)*cos(tht2+tht3));
z3 = z2 + d2 * sin(tht2+tht3);

x4 = x3 + d3 * (cos(tht1)*cos(tht2+tht3+tht4));
y4 = y3 + d3 * (sin(tht1)*cos(tht2+tht3+tht4));
z4 = z3 + d3 * sin(tht2+tht3+tht4);

% ========== VISUALIZACIÓN 3D ==========
fig = figure();
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
view(3);
axis equal;

% Límites del gráfico
a = d1 + d2 + d3;
b = d0 + d1 + d2 + d3;
xlim([-a, a]);
ylim([-a, a]);
zlim([0, b]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Robot 4DOF - Cinemática Inversa');

% Dibujar eslabones
links_x = [x0, x1, x2, x3, x4];
links_y = [y0, y1, y2, y3, y4];
links_z = [z0, z1, z2, z3, z4];

colors = ['r', 'g', 'b', 'c'];
for i = 1:4
    plot3([links_x(i), links_x(i+1)], ...
          [links_y(i), links_y(i+1)], ...
          [links_z(i), links_z(i+1)], ...
          'Color', colors(i), 'LineWidth', 3);
end

% Dibujar articulaciones
scatter3(links_x, links_y, links_z, 100, 'k', 'filled');

% Punto final deseado
scatter3(P(1), P(2), P(3), 150, 'm', 'pentagram', 'filled');
legend('Brazo 1', 'Brazo 2', 'Brazo 3', 'Brazo 4', 'Articulaciones', 'Objetivo');

% ========== FUNCIÓN DH (COMPATIBLE OCTAVE) ==========
function T = DHmethode(a, alpha, d, theta)
    % Matriz de transformación Denavit-Hartenberg
    % Compatible con Octave (simbólico/numérico)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),            d;
         0,          0,                      0,                     1];
end
