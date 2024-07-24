L1 = 12.5; % Longitud del primer eslabón
L2 = 12.5; % Longitud del segundo eslabón

port = "COM3";

% Abre la comunicación serial con el ESP32 una vez al inicio
esp = serialport(port, 115200);

% Inicializa la figura y configura el hold
figure;
hold on;
grid on;
axis equal;
xlim([-L1 - L2, L1 + L2]);
ylim([-L1 - L2, L1 + L2]);
xlabel('X');
ylabel('Y');
title('Cinemática Inversa de un Robot Planar de 2 GDL');
legend('show');

start = 1;
while start
    disp("1) Mover a posicion XY");
    disp("2) Definir Posicion Inicial y Final con Pasos");
    disp("3) Posicion Inicial");
    disp("4) Desconectar");
    n = input('Escoja una opcion: ');
    switch n
        case 1
            prompt = "Insertar Valor en X deseado: ";
            x = input(prompt);  % Coordenada x del objetivo
            prompt1 = "Insertar Valor en Y deseado: ";
            y = input(prompt1);  % Coordenada y del objetivo
            if x < 0
                [theta1, theta2] = cinematica_inversa_2222(x, y, L1, L2);
            else
                [theta1, theta2] = cinematica_inversa_2gdl(x, y, L1, L2);
            end
            graficar_robot(theta1, theta2, L1, L2, x, y);
            serialesp(theta1, theta2, esp);

        case 2
            prompt = "Insertar posicion inicial en X: ";
            x0 = input(prompt);  % Coordenada x del objetivo
            prompt1 = "Insertar posicion inicial en Y: ";
            y0 = input(prompt1);  % Coordenada y del objetivo
            prompt2 = "Insertar posicion final en X: ";
            x1 = input(prompt2);  % Coordenada x del objetivo
            prompt3 = "Insertar posicion final en Y: ";
            y1 = input(prompt3);  % Coordenada y del objetivo
            prompt4 = "Insertar numero de pasos: ";
            num_steps = input(prompt4);

            for i = 0:num_steps
                x = x0 + (x1 - x0) * i / num_steps;
                y = y0 + (y1 - y0) * i / num_steps;
                if x < 0
                    [theta1, theta2] = cinematica_inversa_2222(x, y, L1, L2);
                else
                    [theta1, theta2] = cinematica_inversa_2gdl(x, y, L1, L2);
                end
                graficar_robot(theta1, theta2, L1, L2, x, y);
                serialesp(theta1, theta2, esp);
                pause(0.1); % Pausa para visualizar el movimiento
            end

        case 3
            x = L1+L2;
            y = 0;
            [theta1, theta2] = cinematica_inversa_2gdl(x, y, L1, L2);
            graficar_robot(theta1, theta2, L1, L2, x, y)
            serialesp(theta1, theta2, esp);
        case 4
            start = 0; % Sale del bucle
    end
end

% Cierra la comunicación serial al final del programa
clear esp;

function [theta1, theta2] = cinematica_inversa_2gdl(x, y, L1, L2)
    % Calcula la distancia desde el origen hasta el objetivo
    r = sqrt(x^2 + y^2);
    
    % Verifica si el objetivo está dentro del alcance del robot
    if r > (L1 + L2) || r < abs(L1 - L2)
        error('El objetivo está fuera del alcance del robot.');
    end
    
    % Calcula los ángulos de las articulaciones usando la cinemática inversa
    theta2 = -acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    % Imprime los ángulos
    fprintf('Ángulo 1 (theta1): %.2f grados\n', rad2deg(theta1));
    fprintf('Ángulo 2 (theta2): %.2f grados\n', rad2deg(theta2));
end

function [theta1, theta2] = cinematica_inversa_2222(x, y, L1, L2)
    % Calcula la distancia desde el origen hasta el objetivo
    r = sqrt(x^2 + y^2);
    
    % Verifica si el objetivo está dentro del alcance del robot
    if r > (L1 + L2) || r < abs(L1 - L2)
        error('El objetivo está fuera del alcance del robot.');
    end
    
    % Calcula los ángulos de las articulaciones usando la cinemática inversa
    theta2 = acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    % Imprime los ángulos
    fprintf('Ángulo 1 (theta1): %.2f grados\n', rad2deg(theta1));
    fprintf('Ángulo 2 (theta2): %.2f grados\n', rad2deg(theta2));
end

function graficar_robot(theta1, theta2, L1, L2, x, y)
    % Calcula las posiciones de las articulaciones
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    
    % Grafica el robot
    plot([0, x1, x2], [0, y1, y2], 'o-', 'LineWidth', 2);
    
    % Marca la posición objetivo
    plot(x, y, 'rx', 'LineWidth', 2, 'MarkerSize', 10);
    
    % Actualiza el gráfico
    legend('Robot', 'Objetivo');
end

function serialesp(theta1, theta2, esp)
    % Convierte los ángulos de radianes a grados
    angle1 = rad2deg(theta1);
    angle2 = rad2deg(theta2);
    
    % Mapea los ángulos de 0-180 a 0-440
    mapped_angle1 = map(angle1, 0, 180, 0, 430);
    mapped_angle2 = map(angle2, 0, 180, 0, 430);
    
    % Convierte los ángulos mapeados a una cadena de texto en el formato adecuado
    data = sprintf('g%.2f u%.2f\n', mapped_angle1, mapped_angle2);
    
    % Envía los datos al ESP32
    write(esp, data, "string");
end

function out = map(x, in_min, in_max, out_min, out_max)
    % Mapea un valor x que está en el rango [in_min, in_max] al rango [out_min, out_max]
    out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
end
