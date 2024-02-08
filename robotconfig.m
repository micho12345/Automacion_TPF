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
q0 = [0 pi/2 -pi/2 pi/2 0]; % Posicion inicial
Robot.teach(q0)
% Robot.islimit(q0)

%%% Test Workspace
% Robot.plot(q0)
% hold on
% plot3(xw(:),yw(:),zw(:))
% hold off

%% Test trajectorias
table_start = [150, -100, 200];    % Posición del borde de la hoja (mm)
table_end = [300, 100, 200];       % Posición en diagonal a la posición anterior
r0 = [0, 0, 0];   % Posición inicial de recta (cm)
rf = [15, 20, 0];    % Posición final de recta(cm)
x0 = table_start + r0*10
xf = table_start + rf*10

h1 = sqrt(x0(2)^2 + x0(3)^2);
h2 = sqrt(x0(1)^2 + x0(3)^2);
h3 = sqrt(x0(1)^2 + x0(2)^2);
T0 = transl(x0(1),x0(2),x0(3))*trotz(asin(x0(2)/h3)*180/pi)*troty(acos(x0(3)/h2)*180/pi);
% Tf = T0;
% Tf(:, 4) = [xf(1); xf(2); xf(3); 1]
h1 = sqrt(xf(2)^2 + xf(3)^2);
h2 = sqrt(xf(1)^2 + xf(3)^2);
h3 = sqrt(xf(1)^2 + xf(2)^2);
%Tf = transl(xf(1),xf(2),xf(3))*trotz((xf(2)/xf(1))*180/pi)*troty((xf(3)/xf(2))*180/pi);
Tf = transl(xf(1),xf(2),xf(3))*trotz(asin(xf(2)/h3)*180/pi)*troty(acos(xf(3)/h2)*180/pi);

%[x, xd, xdd] = jtraj(x0, xf, t);  %Calcula las componentes en cada periodo de tiempo
% T1 = transl(x0)
% T2 = transl(xf)
% Q1 = Robot.ikine(T1, 'q0', q1, 'mask', [1 1 1 1 1 0])
% q2 = [0.21, 41.6, -0.576, 1.408, 0]
% Q2 = Robot.ikine(T2, 'q0', q2, 'mask', [1 1 1 1 1 0])
% t = 0:0.15:10;
% T1 = Robot.fkine(Q0);
% T2 = Robot.fkine(Qf);
%traj = jtraj(Q1, Q2, t);

%q1 = [-0.312, 1.61, -1.8256, 1.8, 0];
% T1 = Robot.fkine(q1);
points = 50;
Trot = ctraj(T0, Tf, points);
% r = Trot(:, :, 1)
% T = zeros(4, 4, points);
% for i = 1:length(T)
%     T(:, :, i) = [r(:, 1), r(:, 2), r(:, 3), Trot(:, 4, i)];
% end
traj = Robot.ikine(Tf, 'q0', q0, 'mask', [1 1 1 1 1 0])
%[traj, err, exitflag] = Robot.ikcon(Trot, q1);

% GRÁFICOS
% Rectángulo
xRectangulo = [table_start(1), table_end(1), table_end(1), table_start(1)];
yRectangulo = [table_start(2), table_start(2), table_end(2), table_end(2)];
zRectangulo = [table_start(3), table_start(3), table_start(3), table_start(3)];
fill3(xRectangulo, yRectangulo, zRectangulo, 'g'); % color azul

% Robot
Robot.plot(traj)
hold on

% Línea
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


