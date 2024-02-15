%% TP Final Automación Industrial
% Ignacio Cutignola (59330)
% Olivia De Vincenti (60354)
% Valentino Venier Anache (60097)

close all
clear all
clc

%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 53, 144, 144];
L23 = sqrt(L(2)^2 + L(3)^2);
Lee = 100;
%qlim = {[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};

% Mesa de trabajo
table_start = [210, -100, 180];    % Posición del borde de la hoja (mm)
table_end = [360, 100, 180];       % Posición en diagonal a la posición anterior

%% Test
Robot = createRobot(L, Lee);
q0 = deg2rad([0 90 -90 0]); % Posicion inicial
drawRectangle(table_start, table_end)
Robot.teach(q0)
% Robot.islimit(q0)

%% Test Workspace
thlim = [90, 1, 1, 1];
[xw, yw, zw] = workspace(L, Lee, thlim);
Robot.plot(q0)
hold on
plot3(xw(:),yw(:),zw(:))
hold off

%% Draw Line
r0 = [0, 0, 0];   % Posición inicial de recta (mm)
rf = [150, 200, 0];    % Posición final de recta(mm)
z_offset = 20;
drawLine(Robot, r0, rf, table_start, table_end, z_offset, q0)

%% createrobot: Crea el modelo del brazo WidowX MK-II
    % Recibe:
        % L: las longitudes como fueron definidas en la presentación (vector 1x5)
        % Lee: la longitud entre el último link y el end effector (escalar)
        % qlim: los límites angulares de cada actuador (arreglo de celdas 5x2)
        
function [Robot] = createRobot(L, Lee, qlim)

    N = 4;  % Cantidad de links
    % Tipos de link
    types = {'revolute', 'revolute', 'revolute', 'revolute'};
    L23 = sqrt(L(2)^2 + L(3)^2);
    
    % Parametros DH [[theta, d, a, alpha]]
    %DH = {[0, L(1), 0, 0], [pi/2, 0, 0, pi/2], [0, 0, L23, 0], [-pi/2, 0, L(4), 0], [0, L(5), 0, -pi/2]};
    DH = {[0, L(1), 0, 0], [0, 0, 0, pi/2], [0, 0, L23, 0], [0, 0, L(4), 0]};

    % Creación de Links
    for i = 1:N
        links{i} = Link(DH{i}, 'modified', types{i}); % Vector de estructuras Link
        if exist('qlim','var')     % Definir limites si se proporcionaron
            links{i}.qlim = qlim{i};
        end
    end

    % Creación del Robot
    Rtool = [0 1 0; -1 0 0; 0 0 1];
    Ttool = [L(5); -Lee; 0];
    Tool = [Rtool, Ttool; 0 0 0 1];
    Robot = SerialLink([links{:}], 'tool', Tool, 'name', 'Pacho Norras');
    
end

function drawLine(Robot, r0, rf, table_start, table_end, z_offset, q0)
    x0 = table_start + r0;
    xf = table_start + rf;
    x0_offset = x0 + [0, 0, z_offset];
    xf_offset = xf + [0, 0, z_offset];

    R = [0,0,1; 0,1,0; -1,0,0];
    q1 = move_robot(Robot, x0_offset, x0, R, q0);
    q2 = move_robot(Robot, x0, xf, R, q0);
    q3 = move_robot(Robot, xf, xf_offset, R, q0);

    % GRÁFICOS
    % Rectángulo
    drawRectangle(table_start, table_end)

    % Robot
    Robot.plot(q1)
    Robot.plot(q2)
    Robot.plot(q3)
    hold on

    % Línea
    xline = linspace(x0(1), xf(1), 100);
    yline = linspace(x0(2), xf(2), 100);
    zline = linspace(x0(3), xf(3), 100);
    plot3(xline, yline, zline, 'LineWidth', 2, 'Color', 'r')
    hold off
end

function drawRectangle(table_start, table_end)
    xRect = [table_start(1), table_end(1), table_end(1), table_start(1)];
    yRect = [table_start(2), table_start(2), table_end(2), table_end(2)];
    zRect = [table_start(3), table_start(3), table_start(3), table_start(3)];
    fill3(xRect, yRect, zRect, 'g'); % color azul
end
