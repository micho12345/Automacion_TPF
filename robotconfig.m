%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 53, 144, 144];
Lee = 100;
qlim = {[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};

%% Transformadas
import tramas.*;
syms L1 L2 L3 L4 L5 Leesym
Lsym = [L1, L2, L3, L4, L5];
L23 = sqrt(Lsym(2)^2 + Lsym(3)^2);
syms o1 o2 o3 o4 o5
T01 = tramas(0, 0, o1, Lsym(1));
T12 = tramas(pi/2, 0, o2 + pi/2, 0);
T23 = tramas(0, L23, o3, 0);
T34 = tramas(0, Lsym(4), o4 - pi/2, 0);
T45 = tramas(-pi/2, 0, o5, Lsym(5));
T5ee = tramas(0, Leesym, 0, 0);
Peesym = T01*T12*T23*T34*T45*T5ee(:, 4);

Peesym(1) = - L23*cos(o1)*sin(o2) - L4*cos(o1)*sin(o2+o3) - L5*cos(o1)*sin(o2+o3+o4) - Leesym*(sin(o1)*sin(o5) - cos(o5)*cos(o1)*cos(o2+o3+o4));
Peesym(2) = - L23*sin(o1)*sin(o2) - L4*sin(o1)*sin(o2+o3) - L5*sin(o1)*sin(o2+o3+o4) + Leesym*(cos(o1)*sin(o5) + cos(o5)*sin(o1)*cos(o2+o3+o4));
Peesym(3) = L1 + L23*cos(o2) + L4*cos(o2+o3) + L5*cos(o2+o3+o4) + Leesym*cos(o5)*sin(o2+o3+o4);

%%
Pee = subs(Peesym, Lsym, L);
Pee = subs(Pee, Leesym, Lee);
thlim = [90, 90, 90, 90, 90];
th1 = linspace(-double(thlim(1)),double(thlim(1)),8.0)*pi/180;
th2 = linspace(-double(thlim(2)),double(thlim(2)),8.0)*pi/180;
th3 = linspace(-double(thlim(3)),double(thlim(3)),8.0)*pi/180;
th4 = linspace(-double(thlim(4)),double(thlim(4)),20.0)*pi/180;  
th5 = linspace(-double(thlim(5)),double(thlim(5)),8.0)*pi/180;
[o1, o2, o3, o4, o5] = ndgrid(th1,th2,th3,th4,th5);

%%
try
    xw = - L23*cos(o1)*sin(o2) - L(4)*cos(o1)*sin(o2+o3) - L(5)*cos(o1)*sin(o2+o3+o4) - Lee*(sin(o1)*sin(o5) - cos(o5)*cos(o1)*cos(o2+o3+o4));
    yw = - L23*sin(o1)*sin(o2) - L(4)*sin(o1)*sin(o2+o3) - L(5)*sin(o1)*sin(o2+o3+o4) + Lee*(cos(o1)*sin(o5) + cos(o5)*sin(o1)*cos(o2+o3+o4));
    zw = L(1) + L23*cos(o2) + L(4)*cos(o2+o3) + L(5)*cos(o2+o3+o4) + Lee*cos(o5)*sin(o2+o3+o4);
catch ME
    disp("error")
end

%% Test
Robot = createRobot(L, Lee, qlim);
q0 = [0 pi/2 -pi/2 -pi/2 0]; % Posicion inicial
%Robot.teach(q0)
%Robot.islimit(q0)
Robot.plot(q0)
hold on
plot3(xw,yw,zw)
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
    
    % Parametros DH
    DH = {[0, L(1), 0, 0], [pi/2, 0, 0, pi/2], [0, 0, L23, 0], [-pi/2, 0, L(4), 0], [0, L(5), 0, -pi/2]};   % [[theta, d, a, alpha]]

    % Creación de Links
    for i = 1:N
        links{i} = Link(DH{i}, 'modified', types{i}); % Vector de estructuras Link
        if exist('qlim','var')     % Definir limites si se proporcionaron
            links{i}.qlim = qlim{i};
        end
    end

    % Creación del Robot
    Robot = SerialLink([links{:}], 'tool', transl([Lee, 0, 0]), 'name', 'Pacho Norras');
    
end


