%% Actividad 2
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346

close all; clear; clc;

%% Seleccionar ejercicio
n = input('1. Ejercicio 1\n2. Ejercicio 2\n3. Ejercicio 3\n');
switch(n)
    case 1
        ejercicio_1();
    case 2
        ejercicio_2();
    case 3
        ejercicio_3();
    otherwise
        disp('Opcion no valida');
end

%% Ejercicio 1
function ejercicio_1()
    R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)]; % Funcion para matriz de rotacion

    oRe = R(pi); % Matriz de rotacion del eje coordinado 'o' al eje 'e'
    ote = [2; 2]; % Vector de traslacion del eje coordinado 'o' al eje 'e'
    op = [-2; 4]; % Punto referenciado con respecto al eje 'o'

    Dibujar_Sistema_Referencia_2D(eye(2, 2), [0; 0], 'o') % Dibujar el eje 'o'
    Dibujar_Sistema_Referencia_2D(oRe, ote, 'e') % Dibujar el eje 'e', usando la matriz de rotacion y el vector de traslacion

    Dibujar_Punto_2D(op) % Dibujar el punto 'op' referenciado con respecto al eje 'o'
    
    % Calcular el punto 'op' pero referenciado con respecto al eje 'e'
    % Se multiplica la matriz de rotacion inversa por la diferencia entre el punto, referenciado 
    % con respecto al eje 'o', y el vector de traslacion  
    ep = oRe' * (op - ote);
    
    % Imprimir el resultado
    disp('Referenciado con respecto al eje ''o'':');
    disp(op);
    disp('Referenciado con respecto al eje ''e'':');
    disp(ep);
end

%% Ejercicio 2
function ejercicio_2()
    oRe = [0 -1; 1 0]; % Matriz de rotacion del eje coordinado 'o' al eje 'e'
    ote = [2; 2]; % Vector de traslacion del eje coordinado 'o' al eje 'e'
    r = 2.8284; % Distancia del punto ep al origin del eje coordinado 'e'
    alpha = pi/4; % Angulo del punto ep con respecto al eje x del eje coordinado 'e'

    x = r * cos(alpha); % Calcular la coordenada x del punto ep en el eje coordinado 'e'
    y = r * sin(alpha); % Calcular la coordenada y del punto ep en el eje coordinado 'e'
    ep = [x; y]; % Punto ep referenciado con respecto al eje coordinado 'e'

    Dibujar_Sistema_Referencia_2D(eye(2, 2), [0; 0], 'o') % Dibujar el eje 'o'
    Dibujar_Sistema_Referencia_2D(oRe, ote, 'e') % Dibujar el eje 'e', usando la matriz de rotacion y el vector de traslacion

    % Calcular el punto 'ep' pero referenciado con respecto al eje 'o'
    % Se multiplica la matriz de rotacion por el punto 'ep' y se suma 
    % al vector de traslacion del eje coordinado 'o' al eje 'e'
    op = oRe * ep + ote;
    
    % Dibujar el punto referenciado con respecto al eje 'o'
    Dibujar_Punto_2D(op)
    
    % Imprimir el resultado
    disp('Referenciado con respecto al eje ''e'':');
    disp(ep);
    disp('Referenciado con respecto al eje ''o'':');
    disp(op);
end

%% Ejercicio 3
function ejercicio_3()
    wRe = [0 0 1; 0 1 0; -1 0 0]; % Matriz de rotacion del eje coordinado 'w' al eje 'e'
    wte = [5.0 5.0 2.0]'; % Vector de traslacion del eje coordinado 'w' al eje 'e'
    wTe = [[wRe wte]; 0 0 0 1]; % Matriz homogenea del eje coordinado 'w' al eje 'e'

    wRo = [0 -1 0; 1 0 0; 0 0 1]; % Matriz de rotacion del eje coordinado 'w' al eje 'o'
    wto = [5.0 -5.0 4.0]'; % Vector de traslacion del eje coordinado 'w' al eje 'o'
    wTo = [[wRo wto]; 0 0 0 1]; % Matriz homogenea del eje coordinado 'w' al eje 'o'

    ep = [0 0 -5]'; % El punto referenciado con respecto al eje coordinado 'e'

    % Calcular el punto 'ep' pero referenciado con respecto al eje 'w'
    % Se multiplica la matriz homogenea por el punto 'ep',  
    % concatenado con un 1 para que tambien sea homogenea
    wp = wTe * [ep; 1];

    % Calcular el punto 'wp' pero referenciado con respecto al eje 'o'
    % Se calcula la inversa de la matriz homogenea del eje coordinado 'w' al eje 'o'
    % usando la inversa de la matriz de rotacion y el vector de traslacion
    % del eje coordinado 'w' al eje 'o'
    wTo_i = [[wRo' -wRo' * wto]; 0 0 0 1];
    op = wTo_i * wp;

    Dibujar_Sistema_Referencia_3D(eye(4, 4), 'w') % Dibujar el eje 'w'
    Dibujar_Sistema_Referencia_3D(wTe, 'e') % Dibujar el eje 'e', usando la matriz homogenea correspondiente
    Dibujar_Sistema_Referencia_3D(wTo, 'o') % Dibujar el eje 'o', usando la matriz homogenea correspondiente
    Dibujar_Punto_3D(wp) % Dibujar el punto referenciado con respecto al eje 'o'
    
    % Imprimir el resultado
    disp('Referenciado con respecto al eje ''e'':');
    disp(ep);
    disp('Referenciado con respecto al eje ''w'':');
    disp(wp(1:3));
    disp('Referenciado con respecto al eje ''o'':');
    disp(op(1:3));
end