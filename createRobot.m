%% createrobot: Crea el modelo del brazo WidowX MK-II
    % Recibe:
        % L: las longitudes como fueron definidas en la presentación (vector 1x5)
        % Lee: la longitud entre el último link y el end effector (escalar)
        % qlim: los límites angulares de cada actuador (arreglo de celdas 5x2)
        
function [Robot] = createRobot(L, Lee, tol)

    N = 4;  % Cantidad de links
    % Tipos de link
    types = {'revolute', 'revolute', 'revolute', 'revolute'};
    for i = 1:length(L)
        L(i) = L(i)*(1 + tol*rand(1));
    end
    L
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