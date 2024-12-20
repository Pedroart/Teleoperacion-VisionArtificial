close all
% Datos de ejemplo: scalpelpositions20241029181937
%Data = scalpelpositions20241029181937(939:2032,1:4);
%Data = scalpelpositions20241029152628;
Data = scalpelpositions20241029153020(666:2400,1:4)

% Ajustar x e y
x_data = (Data.x - 0.55)/1.8 ;
y_data = (Data.y -0.1)/1.8;

% Crear z como el índice de cada punto
z_data = 1:length(x_data);  % Esto toma la posición en el vector como índice

% Crear la figura en 3D
figure;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Número de muestra');
title('Movimiento en el espacio 3D (XY con índice Z)');
xlim([-0.1,0.1]);
ylim([-0.1,0.1]);
plot3(x_data, y_data, z_data); % Gráfica en 3D
grid on;
view(3); % Vista en 3D

% Dibujar el círculo en el plano z = 0
theta = linspace(0, 2*pi, 100);
circle_x = 0.05 * cos(theta); % Coordenadas X del círculo
circle_y = 0.05 * sin(theta); % Coordenadas Y del círculo
circle_z = zeros(size(circle_x)); % Coordenadas Z en z = 0
plot3(circle_x, circle_y, circle_z, 'k'); % Círculo en color negro

% Dibujar la cruz dentro del círculo en el plano z = 0
cross_x = [-0.025 0.025 NaN 0 0]; % Coordenadas X de la cruz
cross_y = [0 0 NaN -0.025 0.025]; % Coordenadas Y de la cruz
cross_z = [0 0 NaN 0 0]; % Coordenadas Z en z = 0
plot3(cross_x, cross_y, cross_z, 'k'); % Cruz en color negro

grid on;
view(3); % Vista en 3D
