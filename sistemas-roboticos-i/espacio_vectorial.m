%% Actividad 1
close all; clear all; clc
%% Figura y Componentes
fig = uifigure('Name', 'Actividad 1'); % Crear figura con interfaz grafica
fig.Position = [200 200 500 400]; % Posicion y dimensiones de figura

% Tabla para el primer vector
table1 = uitable(fig, 'Data', zeros(2,1), 'Position', [20 70 125 250]);
set(table1, 'ColumnEditable', true); % Hacer editable

% Tabla para el segundo vector
table2 = uitable(fig, 'Data', zeros(2,1), 'Position', [355 70 125 250]);
set(table2, 'ColumnEditable', true); % Hacer editable

% Caja de texto editable para el no. de filas del primer vector
filas1 = uilabel(fig, 'Text', 'Filas:'); % Etiqueta
filas1.Position = [20 340 50 30]; % Posicion de etiqueta
edt1 = uieditfield(fig, 'numeric', 'Value', 2); % Caja de texto editable con valor inicial de 2
edt1.Position = [70 340 50 30]; % Posicion de caja de texto

% Caja de texto no editable para la norma de el primer vector
norma1 = uilabel(fig, 'Text', 'Norma:'); % Etiqueta
norma1.Position = [20 20 50 30]; % Posicion de etiqueta
norma1Box = uitextarea(fig, 'Value', num2str(0), 'Editable', 'off'); % Caja de texto no editable
norma1Box.Position = [70 20 75 30]; % Posicion de caja

% Caja de texto editable para el no. de filas del segundo vector
filas2 = uilabel(fig, 'Text', 'Filas:');
filas2.Position = [355 340 50 30]; 
edt2 = uieditfield(fig, 'numeric', 'Value', 2);
edt2.Position = [405 340 50 30];

% Caja de texto no editable para la norma del segundo vector
norma2 = uilabel(fig, 'Text', 'Norma:'); 
norma2.Position = [355 20 50 30];
norma2Box = uitextarea(fig, 'Value', num2str(0), 'Editable', 'off');
norma2Box.Position = [405 20 75 30];

% Caja de texto no editable para el producto punto de los dos vectores
producto = uilabel(fig, 'Text', 'Producto interno:'); 
producto.Position = [200 225 100 100];
productoBox = uitextarea(fig, 'Value', num2str(0), 'Editable', 'off');
productoBox.Position = [200 225 100 30];

% Caja de texto no editable para el angulo entre los dos vectores
angulo = uilabel(fig, 'Text', 'Angulo (grados):'); 
angulo.Position = [200 125 100 100];
anguloBox = uitextarea(fig, 'Value', num2str(0), 'Editable', 'off');
anguloBox.Position = [200 125 100 30];
%% Funciones
% Declaracion de funciones para los componentes editables de la figura
table1.CellEditCallback = @(table1, event) calcular(table1, table2, norma1Box, norma2Box, productoBox, anguloBox);
table2.CellEditCallback = @(table2, event) calcular(table1, table2, norma2Box, norma2Box, productoBox, anguloBox);
edt1.ValueChangedFcn = @(edt1, event) filasVector(edt1.Value, edt2.Value, table1, table2, norma1Box, norma2Box, productoBox, anguloBox);
edt2.ValueChangedFcn = @(edt2, event) filasVector(edt1.Value, edt2.Value, table1, table2, norma1Box, norma2Box, productoBox, anguloBox);

% Cambiar el no. de filas de los vectores
% Se ejecuta cada que se cambie el valor en las cajas para el no. de filas
function filasVector(filas1, filas2, table1, table2, norma1Box, norma2Box, productoBox, anguloBox)
    vec1 = table1.Data; vec2 = table2.Data; % Guardar los dos vectores
    
    % Checar el numero de filas que debe tener el primer vector
    if length(vec1) > filas1 % Si debe tener menos filas de los que tiene
        vec1 = vec1(1:filas1); % Se borran los valores de las filas que ya no existen
    elseif length(vec1) < filas1 % Si se quiere agregar mas filas
        vec1 = [vec1', zeros(1, filas1 - length(vec1))]'; % Se concatena un vector de ceros al final del vector original
    end
    set(table1, 'Data', vec1); % Actualizar la tabla del primer vector
    
    % Checar el numero de filas que debe tener el segundo vector
    % Mismo procedimiento que en el primer vector
    if length(vec2) > filas2
        vec2 = vec2(1:filas2);
    elseif length(vec2) < filas2
        vec2 = [vec2', zeros(1, filas2 - length(vec2))]';
    end
    set(table2, 'Data', vec2); % Actualizar la tabla del segundo vector
    
    % Mandar los datos ya actualizados a la funcion calcular
    % Para obtener normas, producto punto y angulo
    calcular(table1, table2, norma1Box, norma2Box, productoBox, anguloBox);
end
% Calcular normas, producto punto y angulo
% Se ejecuta cada que se cambie un valor en las tablas
function calcular(table1, table2, norma1Box, norma2Box, productoBox, anguloBox)
    vec1 = table1.Data; vec2 = table2.Data; % Guardar los vectores
    norma1 = 0; norma2 = 0; producto = 0; angulo = 0; % Inicializar variables
    
    % Calculo de las normas
    % Sumatoria del cuadrado de los elementos de cada vector
    for i = 1:length(vec1)
        norma1 = norma1 + vec1(i)^2;
    end   
    for i = 1:length(vec2)
        norma2 = norma2 + vec2(i)^2;
    end
    % Raiz cuadrada de las sumatorias para obtener las normas
    norma1 = sqrt(norma1); norma2 = sqrt(norma2);
    % Actualizar las cajas de las normas para cada vector
    norma1Box.Value = num2str(norma1);
    norma2Box.Value = num2str(norma2);
    
    % Calculo de producto punto y angulo
    if length(vec1) == length(vec2) % Checar que sean del mismo tamano
        for i = 1:length(vec1) % Calcular producto         
            producto = producto + vec1(i)*vec2(i); % Sumatoria de los productos
        end
        angulo = acosd(producto/(norma1*norma2)); % Calcular el angulo (grados)
        % Actualizar las cajas para el producto y el angulo
        productoBox.Value = num2str(producto);
        anguloBox.Value = num2str(angulo);
    else % Si son de diferentes tamanos
        productoBox.Value = 'Error';
        anguloBox.Value = 'Error';
    end
    % Imprimir en consola
    clc;
    norma1
    norma2
    producto
    angulo
end