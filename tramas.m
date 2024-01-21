function Tij = tramas(alpha, a, theta, d)
    Tij = round_zeros(Rx(alpha)*Dx(a)*Rz(theta)*Dz(d));
end

%% Funciones para tramas
function M = Rx(a)  % USA RADIANES, CONVERTIR DE GRADOS CON deg2rad(a)
    M = [1, 0, 0, 0; 0, cos(a), -sin(a), 0; 0, sin(a), cos(a), 0; 0, 0, 0, 1];
    if ~isnumeric(a)
        c = coeffs(a);
        s = size(c);
        if s(2) == 2
            M = [1, 0, 0, 0; 0, -sign(c(2))*sin(a), -sign(c(2))*cos(a), 0; 0, sign(c(2))*cos(a), -sign(c(2))*sin(a), 0; 0, 0, 0, 1];
        end
    end    
end

function M = Dx(a)
    M = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
end

function M = Rz(a)  % USA RADIANES, CONVERTIR DE GRADOS CON deg2rad(a)
    M = [cos(a), -sin(a), 0, 0; sin(a), cos(a), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    if ~isnumeric(a)
        c = coeffs(a);
        s = size(c);
        if s(2) == 2
            M = [-sign(c(1))*sin(a - c(1)), -sign(c(1))*cos(a - c(1)), 0, 0; sign(c(1))*cos(a - c(1)), -sign(c(1))*sin(a - c(1)), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
            %M = [cos(a - c(1))*cos(c(1)) - sign(c(1))*sin(a - c(1))*sin(c(1)), -sin(a - c(1))*cos(c(1)) - sign(c(1))*cos(a - c(1))*sin(c(1)), 0, 0; sin(a - c(1))*cos(c(1)) + sign(c(1))*cos(a - c(1))*sin(c(1)), cos(a - c(1))*cos(c(1)) - sign(c(1))*sin(a - c(1))*sin(c(1)), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
        end
    end    
end

function M = Dz(a)
    M = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, a; 0, 0, 0, 1];
end

% Round zeros: Si por razones de punto flotante hay números que deberían
% dar cero pero en vez dan ~6.1232e-17, al aplicar esta funcion se redondean
% a cero

function M = round_zeros(M)
    [x, y] = size(M);
    for i = 1:x
        for j = 1:y
            Coef = M(i, j);
            if ~isnumeric(Coef)
                Coef = coeffs(M(i, j));
            end
            if abs(Coef) < 0.0001 % you can change 0.001 to adjust precision
                M(i, j) = subs(M(i, j),Coef,0);
            end
        end
    end
end
