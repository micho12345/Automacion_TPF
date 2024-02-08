clear all
close all
clc
%% Importar imagenes sobre las que trabajamos
im  =iread('c:\00 - Ignacio\Github\Automacion_TPF\Test Images\Foto2.jpg');



%% Aplico la mascara ROJA

%Valores del filtro
red_mask_r_low = 100;
red_mask_r_high = 195;
red_mask_g_low = 10;
red_mask_g_high = 90;
red_mask_b_low = 10;
red_mask_b_high = 90;
idisp(im)

I_R = im;

% Defino los parametros de threshold de R
channel1Min = red_mask_r_low;
channel1Max = red_mask_r_high;

% Defino los parametros de threshold de G
channel2Min = red_mask_g_low;
channel2Max = red_mask_g_high;

% Defino los parametros de threshold de B
channel3Min = red_mask_b_low;
channel3Max = red_mask_b_high;

% Se crea la mascara
sliderBW = (I_R(:,:,1) >= channel1Min ) & (I_R(:,:,1) <= channel1Max) & ...
    (I_R(:,:,2) >= channel2Min ) & (I_R(:,:,2) <= channel2Max) & ...
    (I_R(:,:,3) >= channel3Min ) & (I_R(:,:,3) <= channel3Max);

BW = sliderBW;


% Set background pixels where BW is false to zero.
RedImage = im;
RedImage(repmat(~BW,[1 1 3])) = 0;

Rmon=imono(RedImage);   
Rblack=Rmon>0.9;

% Aplicamos apertura y cierre para rellenar la linea

closed_1 = iclose(Rblack, ones(1, 1));
opened_1 = iopen(closed_1, ones(3, 3));
linea_im = iclose(opened_1, kcircle(20));

idisp(linea_im)
%% Como primer paso vamos a aplicar una mascara VERDE
% Este paso se hace primero para identificar los bordes en donde
% se encuentra la linea roja

%Valores del filtro
green_mask_r_low = 60;
green_mask_r_high = 110;
green_mask_g_low = 60;
green_mask_g_high = 150;
green_mask_b_low = 60;
green_mask_b_high = 150;


I = im;

% Defino los parametros de threshold de R
channel1Min = green_mask_r_low;
channel1Max = green_mask_r_high;

% Defino los parametros de threshold de G
channel2Min = green_mask_g_low;
channel2Max = green_mask_g_high;

% Defino los parametros de threshold de B
channel3Min = green_mask_b_low;
channel3Max = green_mask_b_high;


% Se crea la mascara
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

BW = sliderBW;
GreenImage = im;
GreenImage(repmat(~BW,[1 1 3])) = 0;


% Ahora aplicamos apertura y cierre para tener una buena imagen de los
% bordes, sin tanto ruido
Gmon=imono(GreenImage);   
Gblack=Gmon>0.9;
closed_1 = iclose(Gblack, ones(2, 2));
opened_1 = iopen(closed_1, ones(2, 2));
borde_im = iclose(opened_1, kcircle(10));
idisp(borde_im)

%% Luego identificamos las lineas 

edges = icanny(borde_im);
h = Hough(edges, 'houghthresh', 0.6, 'suppress', 10)
lines = h.lines();

lines = lines.seglength(edges);
k = find( lines.length > 1 & lines.length <= 32);


idisp(borde_im)
lines(k).plot('r')
lines(k)



%% Ahora buscamos las intersecciones

valid_verteces = zeros(2,0);
lines = lines(k)
for i = 1:size(lines,2)
    for j = i:size(lines,2)
        line1 = lines(i);
        line2 = lines(j);
        theta1 = line1.theta;
        theta2 = line2.theta;
        rho1 = line1.rho;
        rho2 = line2.rho;

        A = [sin(theta1),cos(theta1);sin(theta2),cos(theta2)];
        b = [rho1;rho2];
        
        if det(A) ~= 0
            xy = A\b;
            x = xy(1);
            y = xy(2);
            
            
            if x >= 1 && x <= size(borde_im,2) && y >= 1 && y <= size(borde_im,1)
                valid_verteces = [valid_verteces,[x;y]];
            end
          
        
        end
    end
end




idisp(borde_im)
lines.plot('r')
plot_point(valid_verteces,'bold','*')
disp(valid_verteces)
% Filtro puntos


%% Ahora tomamos la imagen original y la acomodamos
    
%Primero acomodalos los vertices en el arreglo para que aparezcan de la
%siguiente manera:
% [INF_IZQ, SUP_IZQ, SUP_DER, INF_DER]

corners = valid_verteces(:, [3 1 2 4]);

% % Mostrar la matriz original y la matriz intercambiada
% disp('Matriz original:');
% disp(valid_verteces);
% disp('Matriz intercambiada:');
% disp(corners);

u_real = 200;
v_real = 150;
precision = 0.1;

target_corners = [1 1 u_real/precision u_real/precision; v_real/precision 1 1 v_real/precision];

%Matriz de homografia
matH = homography(corners, target_corners);

%Corregimos la imagen de la linea a dibujar
test_im = homwarp(matH, im, 'size', [u_real/precision, v_real/precision]);
corrected_im = homwarp(matH, linea_im, 'size', [u_real/precision, v_real/precision]);


idisp(test_im)
idisp(corrected_im)


%% Ahora aplico una mascara roja para esta nueva imagen

[fil,col]=find(corrected_im);          %Obtenemos todos los puntos por los que pasa la linea

points(:,1) = [col(1); fil(1)]
points(:,2) = [col(length(col)); fil(length(fil))]