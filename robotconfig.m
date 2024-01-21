%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 53, 144, 144];
L23 = sqrt(L(2)^2 + L(3)^2);
Lee = 100;
qlim = {[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};

%%
Peesym(1) = L23*cos(o1)*cos(o2) + L4*cos(o1)*cos(o2+o3) + L5*cos(o1)*cos(o2+o3+o4) - Leesym*(sin(o1)*sin(o5) - cos(o5)*cos(o1)*sin(o2+o3+o4));
%Peesym(2) = - L23*sin(o1)*sin(o2) - L4*sin(o1)*sin(o2+o3) - L5*sin(o1)*sin(o2+o3+o4) + Leesym*(cos(o1)*sin(o5) + cos(o5)*sin(o1)*cos(o2+o3+o4));
Peesym(3) = L1 + L23*sin(o2) + L4*sin(o2+o3) + L5*sin(o2+o3+o4) - Leesym*cos(o5)*cos(o2+o3+o4);

%%
%Pee = subs(Peesym, Lsym, L);
%Pee = subs(Pee, Leesym, Lee);
thlim = [90, 1, 1, 1, 1];
th1 = linspace(-double(thlim(1)),double(thlim(1)),90.0)*pi/180;
th2 = linspace(-double(thlim(2)),double(thlim(2)),10.0)*pi/180;
th3 = linspace(-double(thlim(3)),double(thlim(3)),10.0)*pi/180;
th4 = linspace(-double(thlim(4)),double(thlim(4)),40.0)*pi/180;  
th5 = linspace(-double(thlim(5)),double(thlim(5)),10.0)*pi/180;
%[theta1, theta2, theta3, theta4] = ndgrid(th1,th2,th3,th4);
%[o1, o2, o3, o4] = ndgrid(th1,th2,th3,th4);
[o1, o2, o3, o4, o5] = ndgrid(th1,th2,th3,th4,th5);

%%
xw = L23*0.5*(cos(o1-o2) + cos(o1+o2)) + L(4)*0.5*(cos(o1-o2-o3) + cos(o1+o2+o3)) + L(5)*0.5*(cos(o1-o2-o3-o4) + cos(o1+o2+o3+o4)) - Lee*0.5*(cos(o1-o5) - cos(o1+o5) - 0.5*(sin(-o1+o2+o3+o4-o5) + sin(-o1+o2+o3+o4+o5) + sin(o1+o2+o3+o4-o5) + sin(o1+o2+o3+o4+o5)));
yw = L23*0.5*(sin(o1+o2) + sin(o1-o2)) + L(4)*0.5*(sin(o1+o2+o3) + sin(o1-o2-o3)) + L(5)*0.5*( sin(o1+o2+o3+o4) + sin(o1-o2-o3-o4)) + Lee*0.5*((sin(o1+o5) + sin(-o1+o5)) + 0.5*(cos(o1-o2-o3-o4+o5) + cos(o1-o2-o3-o4-o5) - cos(o1+o2+o3+o4-o5) - cos(o1+o2+o3+o4+o5)));
zw = L(1) + L23*sin(o2) + L(4)*sin(o2+o3) + L(5)*sin(o2+o3+o4) - Lee*0.5*(cos(o2+o3+o4-o5) + cos(o2+o3+o4+o5));


% xw = - L23*0.5*(sin(o1+o2) + sin(-o1+o2)) - L(4)*0.5*(sin(o1+o2+o3) + sin(-o1+o2+o3)) - L(5)*0.5*(sin(o1+o2+o3+o4) + sin(-o1+o2+o3+o4)) - Lee*0.5*(cos(o1-o5)-cos(o1+o5)- 0.5*(cos(o1-o2-o3-o4-o5) + cos(o1-o2-o3-o4+o5) + cos(o1+o2+o3+o4-o5) + cos(o1+o2+o3+o4+o5)));
% yw = - L23*0.5*(cos(o1-o2) - cos(o1+o2)) - L(4)*0.5*(cos(o1-o2-o3) - cos(o1+o2+o3)) - L(5)*0.5*(cos(o1-o2-o3-o4) - cos(o1+o2+o3+o4)) + Lee*0.5*(sin(o1+o5) + sin(-o1+o5) + 0.5*(sin(o1+o2+o3+o4+o5) + sin(o1-o2-o3-o4-o5) + sin(o1-o2-o3-o4+o5) + sin(o1+o2+o3+o4-o5)));
% zw = L(1) + L23*cos(o2) + L(4)*cos(o2+o3) + L(5)*cos(o2+o3+o4) + Lee*0.5*(sin(o2+o3+o4+o5) + sin(o2+o3+o4-o5));

%% Test
Robot = createRobot(L, Lee);%, qlim);
q0 = [0 0 0 pi/2 0]; % Posicion inicial
% Robot.teach(q0)
% Robot.islimit(q0)
Robot.plot(q0)
hold on
plot3(xw(:),yw(:),zw(:))
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


