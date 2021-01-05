%% Actividad 2
close all; clear all; clc

%% Parte 1: Figura y Componentes
fig1 = uifigure('Name', 'Actividad 2: Parte 1'); % Crear figura con interfaz grafica
fig1.Position = [200 100 1000 450]; % Posicion y dimensiones de figura

% Tabla para el primer vector o matriz
table1 = uitable(fig1, 'Data', zeros(3,3), 'Position', [20 70 275 250]);
set(table1, 'ColumnEditable', true); % Hacer editable

% Tabla para el segundo vector o matriz
table2 = uitable(fig1, 'Data', zeros(3,3), 'Position', [362.5 70 275 250]);
set(table2, 'ColumnEditable', true); % Hacer editable

% Tabla para el resultado
tablef = uitable(fig1, 'Data', zeros(3,3), 'Position', [705 70 275 250]);
set(tablef, 'ColumnEditable', false); % Hacer no editable

% Caja de texto editable para el no. de filas del primer vector
filas1 = uilabel(fig1, 'Text', 'Filas:  m1='); % Etiqueta
filas1.Position = [20 340 75 30]; % Posicion de etiqueta
edtFilas1 = uieditfield(fig1, 'numeric', 'Value', 3); % Caja de texto editable con valor inicial de 2
edtFilas1.Position = [80 340 50 30]; % Posicion de caja de texto

% Caja de texto editable para el no. de filas del primer vector
columnas1 = uilabel(fig1, 'Text', 'Columnas:  n1='); % Etiqueta
columnas1.Position = [150 340 85 30]; % Posicion de etiqueta
edtColumnas1 = uieditfield(fig1, 'numeric', 'Value', 3); % Caja de texto editable con valor inicial de 2
edtColumnas1.Position = [235 340 50 30]; % Posicion de caja de texto

% Caja de texto editable para el no. de filas del primer vector
filas2 = uilabel(fig1, 'Text', 'Filas:  m2='); % Etiqueta
filas2.Position = [362.5 340 75 30]; % Posicion de etiqueta
edtFilas2 = uieditfield(fig1, 'numeric', 'Value', 3); % Caja de texto editable con valor inicial de 2
edtFilas2.Position = [422.5 340 50 30]; % Posicion de caja de texto

% Caja de texto editable para el no. de filas del primer vector
columnas2 = uilabel(fig1, 'Text', 'Columnas:  n2='); % Etiqueta
columnas2.Position = [492.5 340 85 30]; % Posicion de etiqueta
edtColumnas2 = uieditfield(fig1, 'numeric', 'Value', 3); % Caja de texto editable con valor inicial de 2
edtColumnas2.Position = [577.5 340 50 30]; % Posicion de caja de texto

% Caja de texto editable para el no. de filas del primer vector
equis = uilabel(fig1, 'Text', 'x'); % Etiqueta
equis.Position = [327 175 30 30]; % Posicion de etiqueta

% Caja de texto editable para el no. de filas del primer vector
igual = uilabel(fig1, 'Text', '='); % Etiqueta
igual.Position = [670 175 30 30]; % Posicion de etiqueta

%% Parte 2: Figura y componentes
fig2 = uifigure('Name', 'Actividad 2: Parte 2'); % Crear figura con interfaz grafica
fig2.Position = [150 100 500 500]; % Posicion y dimensiones de figura

% Dropdown menu
% Seleccionar el tipo de transformacion
transDD = uidropdown(fig2, 'Items', {'Escalamiento', 'Rotacion', 'Traslacion'});
transDD.Position = [97.5 380 100 30];

% Cajas de texto editable para los parametros
pxEF = uieditfield(fig2, 'numeric', 'Value', 1, 'Position', [80 325 50 30]);
pyEF = uieditfield(fig2, 'numeric', 'Value', 1, 'Position', [180 325 50 30]);
% Etiquetas
pxLB = uilabel(fig2, 'Text', 'sx =', 'Position', [50 300 50 80]);
pyLB = uilabel(fig2, 'Text', 'sy =', 'Position', [150 300 60 80]); 

% Tabla para la matriz de escalamiento
textMat = {'sx' '0' '0'; '0' 'sy' '0'; '0' '0' '1'};
transTable = uitable(fig2, 'Data', textMat, 'Position', [20 210 275 80]);

% Boton transformacion
transBtn = uibutton(fig2, 'Text', 'Transformar puntos'); % Texto
transBtn.Position = [80 150 150 30]; % Posicion

% Puntos iniciales
p1 = uitable(fig2, 'Data', [1 1 1]', 'Position', [355 310 125 80]);
p2 = uitable(fig2, 'Data', [2 1 1]', 'Position', [355 210 125 80]);
p3 = uitable(fig2, 'Data', [1.5 2 1]', 'Position', [355 110 125 80]);
% Etiquetas
p1LB = uilabel(fig2, 'Text', 'p1 =', 'Position', [330 310 125 80]); 
p2LB = uilabel(fig2, 'Text', 'p2 =', 'Position', [330 210 125 80]);
p3LB = uilabel(fig2, 'Text', 'p3 =', 'Position', [330 110 125 80]);
% Indicar que es multiplicacion
equisLB = uilabel(fig2, 'Text', 'x', 'Position', [310 210 125 80]);

%% Funciones
% Declaracion de funciones para componentes de la interfaz
edtFilas1.ValueChangedFcn = @(edtFilas1, event) dimensiones(edtFilas1.Value, edtColumnas1.Value, table1, table2, tablef, 1);
edtColumnas1.ValueChangedFcn = @(edtColumnas1, event) dimensiones(edtFilas1.Value, edtColumnas1.Value, table1, table2, tablef, 1);
edtFilas2.ValueChangedFcn = @(edtFilas2, event) dimensiones(edtFilas2.Value, edtColumnas2.Value, table1, table2, tablef, 2);
edtColumnas2.ValueChangedFcn = @(edtColumnas2, event) dimensiones(edtFilas2.Value, edtColumnas2.Value, table1, table2, tablef, 2);
table1.CellEditCallback = @(table1, event) calcular(table1.Data, table2.Data, tablef);
table2.CellEditCallback = @(table2, event) calcular(table1.Data, table2.Data, tablef);
transDD.ValueChangedFcn = @(transDD, event) transSelect(transDD.Value, transTable, pxEF, pyEF, pxLB, pyLB);
transBtn.ButtonPushedFcn = @(transBtn, event) scaleBtnPushed(transDD.Value, pxEF.Value, pyEF.Value);

% Parte 1: Cambiar dimensiones de las tablas
function dimensiones(filas, columnas, table1, table2, tablef, opcion)
    if opcion == 1
        mat = table1.Data;
        if filas > size(mat, 1)
            mat = [mat; zeros(filas - size(mat, 1), size(mat, 2))];   
        elseif columnas > size(mat, 2)
            mat = horzcat(mat, zeros(filas, columnas - size(mat, 2)));
        elseif filas < size(mat, 1) || columnas < size(mat, 2)
            mat = mat(1:filas, 1:columnas);
        end
        table1.Data = mat;
    else
        mat = table2.Data;
        if filas > size(mat, 1)
            mat = [mat; zeros(filas - size(mat, 1), size(mat, 2))];   
        elseif columnas > size(mat, 2)
            mat = horzcat(mat, zeros(filas, columnas - size(mat, 2)));
        elseif filas < size(mat, 1) || columnas < size(mat, 2)
            mat = mat(1:filas, 1:columnas);
        end
        table2.Data = mat;
    end
    calcular(table1.Data, table2.Data, tablef);
end
% Parte 1: Calcular nueva matriz
function calcular(mat1, mat2, tablef)
    if size(mat1, 2) == size(mat2, 1)
        matf = multiplicar(mat1, mat2);
        tablef.Data = matf;
    else
        tablef.Data = NaN;
    end
end
% Parte 2: Seleccionar tipo de transformacion
function transSelect(option, transTable, pxEF, pyEF, pxLB, pyLB)
    switch option
        case 'Escalamiento'
            % Valores necesarios para una matriz identidad
            pxEF.Value = 1; pxLB.Text = 'sx ='; 
            set(pyEF, 'visible', 'on', 'Editable', 'on'); 
            % Mostrar segundo parametro
            pyLB.Text = '';
            pyEF.Value = 1; pyLB.Text = 'sy ='; % Cambio de etiqueta
            % Matriz de transformacion
            textMat = {'sx' '0' '0'; '0' 'sy' '0'; '0' '0' '1'};
            transTable.Data = textMat;
        case 'Rotacion'
            % Valores necesarios para una matriz identidad
            pxEF.Value = 0; pxLB.Text = 'tht =';
            % Esconder segundo parametro ya que no es necesario
            set(pyEF, 'visible', 'off'); pyLB.Text = 'en grados';
            % Matriz de transformacion
            textMat = {'cos(tht)' '-sin(tht)' '0'; 'sin(tht)' 'cos(tht)' '0'; '0' '0' '1'};
            transTable.Data = textMat;
        case 'Traslacion'
            % Valores necesarios para una matriz identidad
            pxEF.Value = 0; pxLB.Text = 'tx =';
            % Mostrar segundo parametro
            set(pyEF, 'visible', 'on', 'Editable', 'on');
            pyLB.Text = '';
            pyEF.Value = 0; pyLB.Text = 'ty ='; % Cambio de etiqueta
            % Matriz de transformacion
            textMat = {'1' '0' 'tx'; '0' '1' 'ty'; '0' '0' '1'};
            transTable.Data = textMat;
    end
end
% Parte 2: Transformar puntos
function scaleBtnPushed(option, px, py)
    % Sustituir los parametros en la matriz de transformacion que se eligio
    switch option
        case 'Escalamiento'
            mat = [px 0 0; 0 py 0; 0 0 1];
        case 'Rotacion'
            mat = [cosd(px) -sind(px) 0; sind(px) cosd(px) 0; 0 0 1];
        case 'Traslacion'
            mat = [1 0 px; 0 1 py; 0 0 1];
    end
    % Calcular puntos transformados
    p1 = [1; 1; 1]; p2 = [2; 1; 1]; p3 = [1.5; 2; 1];
    p1f = multiplicar(mat, p1) 
    p2f = multiplicar(mat, p2)
    p3f = multiplicar(mat, p3)
    % Figura nueva para graficar los puntos
    figure('Position', [650 150 500 500]);
    plot([1 2 1.5], [1 1 2], 'ro'); % Puntos originales
    hold on
    plot([p1f(1) p2f(1) p3f(1)], [p1f(2) p2f(2) p3f(2)], 'go'); % Puntos transformados
    axis([-10 10 -10 10]); grid on; legend('Inicial', 'Final');% Limites y cuadricula
    
end
% Multiplicar matrices
function matf = multiplicar(mat1, mat2)
    matf = zeros(size(mat1, 1), size(mat2, 2)); % Inicializacion de nueva matriz
    for i = 1:size(mat1, 1) % Recorrer filas de 1ra matriz
        for j = 1:size(mat2, 2) % Recorrer columnas de 2da matriz
            sum = 0; % Valor que guardara sumatoria
            for k = 1:size(mat2, 1) % Recorrer filas de 2da matriz
                sum = sum + mat1(i, k)*mat2(k, j); % Multiplicar e ir sumando
            end
            matf(i, j) = sum; % Guardar la suma en la matriz nueva
        end
    end
end