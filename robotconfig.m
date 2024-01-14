%% Definimos características del robot

N = 5; % Cantidad de links

% Tipos de link
types = {'revolute', 'revolute', 'revolute', 'revolute', 'revolute'};

% Longitudes en mm
L = [130, 144, 53, 144, 144];
L23 = sqrt(L(2)^2 + L(3)^2)
Lee = 120;

% Parametros DH
% [[theta, d, a, alpha]]
DH = {[0, L(1), 0, 0], [pi/2, 0, 0, pi/2], [0, 0, L23, 0], [-pi/2, 0, L(4), 0], [0, L(5), 0, -pi/2]};

%% Creación de Links
for i = 1:N
    links{i} = Link(DH{i}, 'modified', types{i}); % Vector de estructuras Link
end

%% Creación del Robot
Robot = SerialLink([links{:}], 'tool', transl([0, 0, Lee]), 'name', 'Pacho Norras');
q0 = [0 pi/2 -pi/2 -pi/2 0]; % Posicion inicial
%q0 = [0 0 0 0 0]; % Posicion inicial

%% Test
Robot.teach(q0)
