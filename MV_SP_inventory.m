clear
close all
%% Load Calibration Parameters
% Import the file
newData1 = load('-mat', 'MV_calibration_parameters.mat');
% Create new variables in the base workspace from those fields.
vars = fieldnames(newData1);
for i = 1:length(vars)
    assignin('base', vars{i}, newData1.(vars{i}));
end
%% Read the Image of Objects to Be Measured
tic
magnification = 50;
imOrig = imread('Image499.png');
figure; imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');
%% Undistort the Image
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');
%% Compute Extrinsics
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints
% Compute rotation and translation of the camera.
[R, T] = extrinsics(imagePoints, worldPoints, cameraParams);
%% Create ROI binary mask
vertices = [(imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160);...
    (imagePoints(1,1)-newOrigin(1)+330) (imagePoints(1,2)-newOrigin(2)-230);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-350);...
    (imagePoints(1,1)-newOrigin(1)+560) (imagePoints(1,2)-newOrigin(2)-320);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-150);...
    (imagePoints(1,1)-newOrigin(1)+260) (imagePoints(1,2)-newOrigin(2)-80);...
    (imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160)];
poly = drawpolygon('Position',vertices);
mask = createMask(poly);
%% Colour segmentation of 4 part types in LAB colour space
imLab = rgb2lab(im);

%red and green
imMask_A = imLab(:,:,2) .* mask;
t_A = graythresh(imMask_A);
imMask_A_red = (imMask_A>t_A+10);
imMask_A_green = (imMask_A<t_A-10);
figure;imshow(imMask_A_red,'InitialMagnification', magnification)
title('Red Legos');
figure;imshow(imMask_A_green,'InitialMagnification', magnification)
title('Green Legos');

%beige
imMask_L = imLab(:,:,1) .* mask;
imMask_B = imLab(:,:,3) .* mask;
max_L = max(imMask_L,[],'all');
max_B = max(imMask_B,[],'all');
t_L = 0.67*max_L;
t_B = 0.54*max_B;
imMask_L_bright = (imMask_L>t_L);
figure;imshow(imMask_L_bright,'InitialMagnification', magnification)
title('Bright Legos');
imMask_B_yellow = (imMask_B>t_B);
figure;imshow(imMask_B_yellow,'InitialMagnification', magnification)
title('Yellow Legos');
imMask_beige=imMask_L_bright;
imMask_beige(imMask_B_yellow==0)=0;
figure;imshow(imMask_beige,'InitialMagnification', magnification)
title('Bright Yellow Legos');

%white
t_L2 = 0.8*max_L;
imMask_white = (imMask_L>t_L2);
figure;imshow(imMask_white,'InitialMagnification', magnification)
title('White Legos');
%% Detect Legos
% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true); %adjust blob area level

[areas_red, boxes_red] = step(blobAnalysis, imMask_A_red);
% Sort connected components in descending order by area
[~, idx_red] = sort(areas_red, 'Descend');
% Get the n largest components.
boxes_red = double(boxes_red(idx_red(1), :));
% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedLegos = imresize(im, scale);
% Insert labels for the Legos.
imDetectedLegos = insertObjectAnnotation(imDetectedLegos, 'rectangle', ...
    scale * boxes_red, 'Red 2x3 Brick');

[areas_green, boxes_green] = step(blobAnalysis, imMask_A_green);
% Sort connected components in descending order by area
[~, idx_green] = sort(areas_green, 'Descend');
% Get the n largest components.
boxes_green = double(boxes_green(idx_green(1), :));
% Insert labels for the Legos.
imDetectedLegos = insertObjectAnnotation(imDetectedLegos, 'rectangle', ...
    scale * boxes_green, 'Green 2x6 Plate');

[areas_beige, boxes_beige] = step(blobAnalysis, imMask_beige);
% Sort connected components in descending order by area
[~, idx_beige] = sort(areas_beige, 'Descend');
% Get the n largest components.
boxes_beige = double(boxes_beige(idx_beige(1), :));
% Insert labels for the Legos.
imDetectedLegos = insertObjectAnnotation(imDetectedLegos, 'rectangle', ...
    scale * boxes_beige, 'Beige 2x10 Plate');

[areas_white, boxes_white] = step(blobAnalysis, imMask_white);
% Sort connected components in descending order by area
[~, idx_white] = sort(areas_white, 'Descend');
% Get the n largest components.
boxes_white = double(boxes_white(idx_white(1), :));
% Insert labels for the Legos.
imDetectedLegos = insertObjectAnnotation(imDetectedLegos, 'rectangle', ...
    scale * boxes_white, 'White 2x6 Brick');

figure; imshow(imDetectedLegos);
title('Detected Legos');
%% Measure angle of each colour Legos
% theta_red = atan(boxes_red(4)/boxes_red(3));
% theta_red_deg = theta_red*180/pi;
% 
% theta_green = atan(boxes_green(4)/boxes_green(3));
% theta_green_deg = theta_green*180/pi;
% 
% theta_beige = atan(boxes_beige(4)/boxes_beige(3));
% theta_beige_deg = theta_beige*180/pi;
% 
% theta_white = atan(boxes_white(4)/boxes_white(3));
% theta_white_deg = theta_white*180/pi;
% 
% if (50<theta_red_deg)&&(theta_red_deg)<56
%     fprintf('Slide 1 = red \n');
% end
% if (40<theta_red_deg)&&(theta_red_deg)<46
%     fprintf('Slide 2 = red \n');
% end
% if (30<theta_red_deg)&&(theta_red_deg)<36
%     fprintf('Slide 3 = red \n');
% end
% if (20<theta_red_deg)&&(theta_red_deg)<26
%     fprintf('Slide 4 = red \n');
% end
% 
% if (50<theta_white_deg)&&(theta_white_deg)<56
%     fprintf('Slide 1 = white \n');
% end
% if (40<theta_white_deg)&&(theta_white_deg)<46
%     fprintf('Slide 2 = white \n');
% end
% if (30<theta_white_deg)&&(theta_white_deg)<36
%     fprintf('Slide 3 = white \n');
% end
% if (20<theta_white_deg)&&(theta_white_deg)<26
%     fprintf('Slide 4 = white \n');
% end
% 
% if (50<theta_green_deg)&&(theta_green_deg)<56
%     fprintf('Slide 1 = green \n');
% end
% if (40<theta_green_deg)&&(theta_green_deg)<46
%     fprintf('Slide 2 = green \n');
% end
% if (30<theta_green_deg)&&(theta_green_deg)<36
%     fprintf('Slide 3 = green \n');
% end
% if (20<theta_green_deg)&&(theta_green_deg)<26
%     fprintf('Slide 4 = green \n');
% end
% 
% 
% if (50<theta_beige_deg)&&(theta_beige_deg)<56
%     fprintf('Slide 1 = beige \n');
% end
% if (40<theta_beige_deg)&&(theta_beige_deg)<46
%     fprintf('Slide 2 = beige \n');
% end
% if (30<theta_beige_deg)&&(theta_beige_deg)<36
%     fprintf('Slide 3 = beige \n');
% end
% if (20<theta_beige_deg)&&(theta_beige_deg)<26
%     fprintf('Slide 4 = beige \n');
% end

%% Detect LH coordinates of each part type regions and compare to slide positions
xL_red = boxes_red(1);
xL_green = boxes_green(1);
xL_white = boxes_white(1);
xL_beige = boxes_beige(1);

if (1500<xL_red)&&(xL_red<1550)
   fprintf('Slide 4 = red \n'); 
end
if (1500<xL_green)&&(xL_green<1550)
   fprintf('Slide 4 = green \n'); 
end
if (1500<xL_white)&&(xL_white<1550)
   fprintf('Slide 4 = white \n'); 
end
if (1500<xL_beige)&&(xL_beige<1550)
   fprintf('Slide 4 = beige \n'); 
end

if (1450<xL_red)&&(xL_red<1500)
   fprintf('Slide 3 = red \n'); 
end
if (1450<xL_green)&&(xL_green<1500)
   fprintf('Slide 3 = green \n'); 
end
if (1450<xL_white)&&(xL_white<1500)
   fprintf('Slide 3 = white \n'); 
end
if (1450<xL_beige)&&(xL_beige<1500)
   fprintf('Slide 3 = beige \n'); 
end

if (1400<xL_red)&&(xL_red<1450)
   fprintf('Slide 2 = red \n'); 
end
if (1400<xL_green)&&(xL_green<1450)
   fprintf('Slide 2 = green \n'); 
end
if (1400<xL_white)&&(xL_white<1450)
   fprintf('Slide 2 = white \n'); 
end
if (1400<xL_beige)&&(xL_beige<1450)
   fprintf('Slide 2 = beige \n'); 
end

if (1350<xL_red)&&(xL_red<1400)
   fprintf('Slide 1 = red \n'); 
end
if (1350<xL_green)&&(xL_green<1400)
   fprintf('Slide 1 = green \n'); 
end
if (1350<xL_white)&&(xL_white<1400)
   fprintf('Slide 1 = white \n'); 
end
if (1350<xL_beige)&&(xL_beige<1400)
   fprintf('Slide 1 = beige \n'); 
end