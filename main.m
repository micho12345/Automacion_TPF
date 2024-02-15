%% TP Final Automación Industrial 2023
% Ignacio Cutignola (59330)
% Olivia De Vincenti (60354)
% Valentino Venier Anache (60097)

close all
clear all
clc

%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 50, 144, 144];
Lee = 100;
Ltol = 0;

% Mesa de trabajo
table_start = [210, -100, 180];    % Posición del borde de la hoja (mm)
table_end = [360, 100, 180];       % Posición en diagonal a la posición anterior
table_size = [150, 200];           % Tamaño de la hoja

%% Crear manipulador
Robot = createRobot(L, Lee, Ltol);
q0 = deg2rad([0 90 -90 0]);    % Posicion inicial
drawRectangle(table_start, table_end)
Robot.teach(q0)

%% Espacio de trabajo
thlim = [180, 110, 90, 100];
%thlim = [50, 60, 60, 60];
[xw, yw, zw] = workspace(L, Lee, thlim);
drawRectangle(table_start, table_end)
Robot.plot(q0)
hold on
plot3(xw(:),yw(:),zw(:))
hold off

%% Visión: Busco linea
points = findLine();
r0 = [points(2), points(1), 0];
rf = [points(4), points(3), 0];
close all

%% Dibujo linea con manipulador
z_offset = 20;
drawLine(Robot, r0, rf, table_start, table_end, z_offset, q0)
