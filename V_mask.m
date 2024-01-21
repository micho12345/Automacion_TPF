clear all
close all
clc
%% Importar imagenes sobre las que trabajamos
im  =iread('c:\00 - Ignacio\Github\Automacion_TPF\Test Images\Foto1.jpg');

%% Aplico la mascara ROJA

%Valores del filtro
red_mask_r_low = 100;
red_mask_r_high = 195;
red_mask_g_low = 50;
red_mask_g_high = 90;
red_mask_b_low = 50;
red_mask_b_high = 90;


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

closed_1 = iclose(Rblack, ones(2, 2));
opened_1 = iopen(closed_1, ones(2, 2));
linea_im = iclose(opened_1, kcircle(15));

idisp(linea_im)


%% Aplico la mascara VERDE

%Valores del filtro
green_mask_r_low = 60;
green_mask_r_high = 110;
green_mask_g_low = 60;
green_mask_g_high = 150;
green_mask_b_low = 60;
green_mask_b_high = 150;



% Convert RGB image to chosen color space
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



% Set background pixels where BW is false to zero.
GreenImage = im;
GreenImage(repmat(~BW,[1 1 3])) = 0;

Gmon=imono(GreenImage);   
Gblack=Gmon>0.9;

% Aplicamos apertura y cierre para rellenar la linea

closed_1 = iclose(Gblack, ones(2, 2));
opened_1 = iopen(closed_1, ones(2, 2));
borde_im = iclose(opened_1, kcircle(15));

idisp(borde_im)