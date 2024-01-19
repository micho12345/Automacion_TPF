clear all
close all
clc
%% Importar imagenes sobre las que trabajamos
im  =iread('c:\00 - Ignacio\Github\Automacion_TPF\Test Images\Foto1.jpg');

idisp(im)

%Filtering properties
red_mask_r_low = 100;
red_mask_r_high = 195;
red_mask_g_low = 50;
red_mask_g_high = 90;
red_mask_b_low = 50;
red_mask_b_high = 90;

green_mask_r_low = 10;
green_mask_r_high = 100;
green_mask_g_low = 10;
green_mask_g_high = 150;
green_mask_b_low = 10;
green_mask_b_high = 150;


% Convert RGB image to chosen color space
I = im;

% Define thresholds for channel 1 based on histogram settings
channel1Min = red_mask_r_low;
channel1Max = red_mask_r_high;

% Define thresholds for channel 2 based on histogram settings
channel2Min = red_mask_g_low;
channel2Max = red_mask_g_high;

% Define thresholds for channel 3 based on histogram settings
channel3Min = red_mask_b_low;
channel3Max = red_mask_b_high;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = im;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

idisp(maskedRGBImage)