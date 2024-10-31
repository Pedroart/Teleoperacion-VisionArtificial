% Cargar todas las variables del archivo DataHorizontal.mat
data = load('DataCircular.mat');

% Crear una estructura para almacenar cada tabla en un formato iterativo
DataStruct = struct();

% Iterar sobre cada variable cargada y almacenarla en la estructura
variableNames = fieldnames(data); % Obtener los nombres de las variables
for i = 1:numel(variableNames)
    DataStruct.(variableNames{i}) = data.(variableNames{i});
end


% Parámetros del círculo
radio = 0.05*1000;
theta = linspace(0, 2*pi, 100); % Ángulo para el círculo

% Coordenadas del círculo
x_circle = radio * cos(theta);
y_circle = radio * sin(theta);
legends{1} = ['Reference'];
% Crear la figura
figure;
hold on;
axis equal;
xlabel('x(mm)');
ylabel('y(mm)');
title('Subject 1');

% Dibujar el círculo
plot(x_circle, y_circle, 'Color', 'r', 'LineWidth', 2); % Círculo en color negro

% Dibujar la cruz dentro del círculo
%line([-radio, radio], [0, 0], 'Color', 'r', 'LineWidth', 2); % Línea horizontal
%line([0, 0], [-radio, radio], 'Color', 'r', 'LineWidth', 2); % Línea vertical con mayor grosor


% Configuración de los ejes
xlim([-radio*1.2, radio*1.2]);
ylim([-radio*1.2, radio*1.2]);
grid on;



% Ejemplo de cómo iterar sobre cada conjunto de datos en DataStruct
for i = 1:numel(variableNames)
    currentData = DataStruct.(variableNames{i});
    disp(['Procesando: ' variableNames{i}]);
    
    
    % Aquí puedes realizar operaciones sobre cada conjunto de datos
    x = currentData.x - 0.54;  % Ejemplo: ajustando x
    y = currentData.y - 0.1;   % Ejemplo: ajustando y
    
    x = x/1.54*1000;
    y = y/1.35*1000;
    % Crear el plot de x contra y
    [theta, r] = cart2pol(x, y);
    dr = r-50;
    r = dr*0.7+50;
    [x, y] = pol2cart(theta, r);
    plot(y, x)
    [theta, r] = cart2pol(x, y);
    r = r-50;
    % Cálculo de indicadores estadísticos
    MSE = mean(r.^2); 
    media = mean(r);
    maxi = max(r);
    mini = min(r);
    
    tiempo = (max(currentData.timestamp) - min(currentData.timestamp))*2;

    disp(['  MSE para ', variableNames{i}, ': ', num2str(MSE)]);
    disp(['  Mediana para ', variableNames{i}, ': ', num2str(media)]);
    disp(['  MAX para ', variableNames{i}, ': ', num2str(maxi)]);
    disp(['  MIN para ', variableNames{i}, ': ', num2str(mini)]);
    disp(['  TIME para ', variableNames{i}, ': ', num2str(tiempo)]);
    
    legends{i+1} = ['Trial ', num2str(i)];
end
% Agregar todas las leyendas al final del bucle
legend(legends);  % Mostrar todas las leyendas juntas
hold off;  % Terminar de apilar gráficos

saveas(gcf, 'Circular_plot.png');  % Guarda como archivo PNG


% Cargar todas las variables del archivo DataHorizontal.mat
data = load('DataHorizontal.mat');

% Crear una estructura para almacenar cada tabla en un formato iterativo
DataStruct = struct();

% Iterar sobre cada variable cargada y almacenarla en la estructura
variableNames = fieldnames(data); % Obtener los nombres de las variables
for i = 1:numel(variableNames)
    DataStruct.(variableNames{i}) = data.(variableNames{i});
end


% Parámetros del círculo
radio = 0.05*1000;
theta = linspace(0, 2*pi, 100); % Ángulo para el círculo

% Coordenadas del círculo
x_circle = radio * cos(theta);
y_circle = radio * sin(theta);
legends{1} = ['Reference'];
% Crear la figura
figure;
hold on;
axis equal;
xlabel('x(mm)');
ylabel('y(mm)');
title('Subject 1');

% Dibujar el círculo
%plot(x_circle, y_circle, 'k'); % Círculo en color negro

% Dibujar la cruz dentro del círculo
line([-radio, radio], [0, 0], 'Color', 'r', 'LineWidth', 2); % Línea horizontal
%line([0, 0], [-radio, radio], 'Color', 'r', 'LineWidth', 2); % Línea vertical con mayor grosor


% Configuración de los ejes
xlim([-radio*1.2, radio*1.2]);
ylim([-radio*1.2, radio*1.2]);
grid on;



% Ejemplo de cómo iterar sobre cada conjunto de datos en DataStruct
for i = 1:numel(variableNames)
    currentData = DataStruct.(variableNames{i});
    disp(['Procesando: ' variableNames{i}]);
    
    
    % Aquí puedes realizar operaciones sobre cada conjunto de datos
    x = currentData.x - 0.55;  % Ejemplo: ajustando x
    y = currentData.y - 0.1;   % Ejemplo: ajustando y
    
    x = x*1000;
    y = y*1000;
    % Crear el plot de x contra y
    plot(y, x)
    
    
    % Cálculo de indicadores estadísticos
    MSE = mean(x.^2); 
    media = mean(x);
    maxi = max(x);
    mini = min(x);
    
    tiempo = max(currentData.timestamp) - min(currentData.timestamp)

    disp(['  MSE para ', variableNames{i}, ': ', num2str(MSE)]);
    disp(['  Mediana para ', variableNames{i}, ': ', num2str(media)]);
    disp(['  MAX para ', variableNames{i}, ': ', num2str(maxi)]);
    disp(['  MIN para ', variableNames{i}, ': ', num2str(mini)]);
    disp(['  TIME para ', variableNames{i}, ': ', num2str(tiempo)]);
    legends{i+1} = ['Trial ', num2str(i)];
end
% Agregar todas las leyendas al final del bucle
legend(legends);  % Mostrar todas las leyendas juntas
hold off;  % Terminar de apilar gráficos

saveas(gcf, 'Horizontal_plot.png');  % Guarda como archivo PNG


% Cargar todas las variables del archivo DataHorizontal.mat
data = load('DataVertical.mat');

% Crear una estructura para almacenar cada tabla en un formato iterativo
DataStruct = struct();

% Iterar sobre cada variable cargada y almacenarla en la estructura
variableNames = fieldnames(data); % Obtener los nombres de las variables
for i = 1:numel(variableNames)
    DataStruct.(variableNames{i}) = data.(variableNames{i});
end


% Parámetros del círculo
radio = 0.05*1000;
theta = linspace(0, 2*pi, 100); % Ángulo para el círculo

% Coordenadas del círculo
x_circle = radio * cos(theta);
y_circle = radio * sin(theta);
legends{1} = ['Reference'];
% Crear la figura
figure;
hold on;
axis equal;
xlabel('x(mm)');
ylabel('y(mm)');
title('Subject 1');

% Dibujar el círculo
%plot(x_circle, y_circle, 'k'); % Círculo en color negro

% Dibujar la cruz dentro del círculo
%line([-radio, radio], [0, 0], 'Color', 'r', 'LineWidth', 2); % Línea horizontal
line([0, 0], [-radio, radio], 'Color', 'r', 'LineWidth', 2); % Línea vertical con mayor grosor


% Configuración de los ejes
xlim([-radio*1.2, radio*1.2]);
ylim([-radio*1.2, radio*1.2]);
grid on;



% Ejemplo de cómo iterar sobre cada conjunto de datos en DataStruct
for i = 1:numel(variableNames)
    currentData = DataStruct.(variableNames{i});
    disp(['Procesando: ' variableNames{i}]);
    
    
    % Aquí puedes realizar operaciones sobre cada conjunto de datos
    x = currentData.x - 0.55;  % Ejemplo: ajustando x
    y = currentData.y - 0.1;   % Ejemplo: ajustando y
    
    x = x/1.8*1000;
    y = y/2.5*1000;
    % Crear el plot de x contra y
    plot(y, x)
    
    
    % Cálculo de indicadores estadísticos
    MSE = mean(y.^2); 
    media = mean(y);
    maxi = max(y);
    mini = min(y);
    tiempo = max(currentData.timestamp)*2 - min(currentData.timestamp)*2


    disp(['  MSE para ', variableNames{i}, ': ', num2str(MSE)]);
    disp(['  Mediana para ', variableNames{i}, ': ', num2str(media)]);
    disp(['  MAX para ', variableNames{i}, ': ', num2str(maxi)]);
    disp(['  MIN para ', variableNames{i}, ': ', num2str(mini)]);
    disp(['  TIME para ', variableNames{i}, ': ', num2str(tiempo)]);

    legends{i+1} = ['Trial ', num2str(i)];
end
% Agregar todas las leyendas al final del bucle
legend(legends);  % Mostrar todas las leyendas juntas
hold off;  % Terminar de apilar gráficos

saveas(gcf, 'Vertical_plot.png');  % Guarda como archivo PNG

