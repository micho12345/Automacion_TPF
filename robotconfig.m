%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 53, 144, 144];
L23 = sqrt(L(2)^2 + L(3)^2);
Lee = 100;
qlim = {[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};

%% Workspace
thlim = [90, 90, 90, 90, 90];
th1 = linspace(-double(thlim(1)),double(thlim(1)),90.0)*pi/180;
th2 = linspace(-double(thlim(2)),double(thlim(2)),10.0)*pi/180;
th3 = linspace(-double(thlim(3)),double(thlim(3)),10.0)*pi/180;
th4 = linspace(-double(thlim(4)),double(thlim(4)),40.0)*pi/180;  
th5 = linspace(-double(thlim(5)),double(thlim(5)),10.0)*pi/180;
[o1, o2, o3, o4, o5] = ndgrid(th1,th2,th3,th4,th5);

%%
xw = L23*0.5*(cos(o1-o2) + cos(o1+o2)) + L(4)*0.5*(cos(o1-o2-o3) + cos(o1+o2+o3)) + L(5)*0.5*(cos(o1-o2-o3-o4) + cos(o1+o2+o3+o4)) - Lee*0.5*(cos(o1-o5) - cos(o1+o5) - 0.5*(sin(-o1+o2+o3+o4-o5) + sin(-o1+o2+o3+o4+o5) + sin(o1+o2+o3+o4-o5) + sin(o1+o2+o3+o4+o5)));
yw = L23*0.5*(sin(o1+o2) + sin(o1-o2)) + L(4)*0.5*(sin(o1+o2+o3) + sin(o1-o2-o3)) + L(5)*0.5*( sin(o1+o2+o3+o4) + sin(o1-o2-o3-o4)) + Lee*0.5*((sin(o1+o5) + sin(-o1+o5)) + 0.5*(cos(o1-o2-o3-o4+o5) + cos(o1-o2-o3-o4-o5) - cos(o1+o2+o3+o4-o5) - cos(o1+o2+o3+o4+o5)));
zw = L(1) + L23*sin(o2) + L(4)*sin(o2+o3) + L(5)*sin(o2+o3+o4) - Lee*0.5*(cos(o2+o3+o4-o5) + cos(o2+o3+o4+o5));

%% Test
Robot = createRobot(L, Lee);%, qlim);
q0 = [0 0 0 pi/2 0]; % Posicion inicial
% Robot.teach(q0)
% Robot.islimit(q0)

%%% Test Workspace
% Robot.plot(q0)
% hold on
% plot3(xw(:),yw(:),zw(:))
% hold off

%% Test trajectorias
table_start = [250, -100, L(1)];    % Posición del borde de la hoja (mm)
table_end = [400, 100, L(1)];       % Posición en diagonal a la posición anterior
r0 = [0, 0, 0];   % Posición inicial de recta (cm)
rf = [15, 20, 0];    % Posición final de recta(cm)
x0 = coor_table + r0*10
xf = coor_table + rf*10
t = 0:0.15:10;
[x, xd, xdd] = jtraj(x0, xf, t);  %Calcula las componentes en cada periodo de tiempo
T = transl(x);

% hipotenusa = sqrt(x0(1)^2 + x0(2)^2);
% sinalpha = x0(2)/hipotenusa;
% cosalpha = x0(1)/hipotenusa;
% R0 = [cosalpha, -sinalpha,  0; 
%       sinalpha, cosalpha,   0; 
%       0,        0,          1];
% %R0 = R0 * [0,0,-1; 0,1,0; 1,0,0];
% T0 = [R0, x0'; 0, 0, 0, 1];
% hipotenusa = sqrt(xf(1)^2 + xf(2)^2);
% sinalpha = xf(2)/hipotenusa;
% cosalpha = xf(1)/hipotenusa;
% Rf = [cosalpha, -sinalpha,  0; 
%       sinalpha, cosalpha,   0; 
%       0,        0,          1];
% %Rf = Rf * [0,0,-1; 0,1,0; 1,0,0];
% Tf = [Rf, xf'; 0, 0, 0, 1];
% 
% T = ctraj(T0, Tf, 100);
q_traj = Robot.ikine(T, 'q0', q0, 'mask',[1 1 1 0 0 0], 'noshort', 'pinv');
% RECTÁNGULO
xRectangulo = [table_start(1), table_end(1), table_end(1), table_start(1)];
yRectangulo = [table_start(2), table_start(2), table_end(2), table_end(2)];
zRectangulo = [table_start(3), table_start(3), table_start(3), table_start(3)];
fill3(xRectangulo, yRectangulo, zRectangulo, 'g'); % color azul
Robot.plot(q_traj)
hold on
% LINEA
xline = linspace(x0(1), xf(1), 100);
yline = linspace(x0(2), xf(2), 100);
zline = linspace(x0(3), xf(3), 100);
plot3(xline, yline, zline, 'LineWidth', 2, 'Color', 'r')
hold off

%% createrobot: Crea el modelo del brazo WidowX MK-II
    % Recibe:
        % L: las longitudes como fueron definidas en la presentación (vector 1x5)
        % Lee: la longitud entre el último link y el end effector (escalar)
        % qlim: los límites angulares de cada actuador (arreglo de celdas 5x2)
        
function [Robot] = createRobot(L, Lee, qlim)

    N = 5;  % Cantidad de links
    % Tipos de link
    types = {'revolute', 'revolute', 'revolute', 'revolute', 'revolute'};
    L23 = sqrt(L(2)^2 + L(3)^2);
    
    % Parametros DH [[theta, d, a, alpha]]
    %DH = {[0, L(1), 0, 0], [pi/2, 0, 0, pi/2], [0, 0, L23, 0], [-pi/2, 0, L(4), 0], [0, L(5), 0, -pi/2]};
    DH = {[0, L(1), 0, 0], [0, 0, 0, pi/2], [0, 0, L23, 0], [pi/2, 0, L(4), 0], [0, L(5), 0, pi/2]};

    % Creación de Links
    for i = 1:N
        links{i} = Link(DH{i}, 'modified', types{i}); % Vector de estructuras Link
        if exist('qlim','var')     % Definir limites si se proporcionaron
            links{i}.qlim = qlim{i};
        end
    end

    % Creación del Robot
    Robot = SerialLink([links{:}], 'tool', transl([-Lee, 0, 0]), 'name', 'Pacho Norras');
    
end


