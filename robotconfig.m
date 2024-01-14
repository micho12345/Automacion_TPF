%% Definimos características del robot

% Longitudes en mm
L = [130, 144, 53, 144, 144];
Lee = 100;
qlim = {[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};

%% Test
Robot = createRobot(L, Lee, qlim);
q0 = [0 pi/2 -pi/2 -pi/2 0]; % Posicion inicial
Robot.teach(q0)
Robot.islimit(q0)

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
    Robot = SerialLink([links{:}], 'tool', transl([0, 0, Lee]), 'name', 'Pacho Norras');
    
end
    