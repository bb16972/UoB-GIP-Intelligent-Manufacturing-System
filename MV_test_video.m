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
%% Read the Image to be Processed
magnification = 50;
% For live stream from webcam
% cam = webcam(2); 
% img = snapshot(cam);
% For test video input
vidObj = VideoReader('TEST_VIDEO.mp4');
% figure
frame1 = read(vidObj, round(vidObj.NumFrames*4/49)); %then AP and SPI after 4s
% imshow(frame1);
% figure
frame2 = read(vidObj, round(vidObj.NumFrames*32/49)); %then SPI after 32s
% imshow(frame2); 
% figure
frame3 = read(vidObj, round(vidObj.NumFrames*47/49)); %then AP after 47s
% imshow(frame3); 
%% Frame 1
[im, newOrigin] = undistortImage(frame1, cameraParams, 'OutputView', 'full');

figure;
imshow(im, 'InitialMagnification', magnification);

%AP
imHSV = rgb2hsv(im);
rectsize = [400 300];
rect = drawrectangle('Position',[(imagePoints(1,1)-newOrigin(1)-0.5*(rectsize(1))),imagePoints(1,2)-newOrigin(2)-1.3*(rectsize(2)),rectsize(1),rectsize(2)]);

mask = createMask(rect,imHSV);
imMask = imHSV(:,:,3) .* mask;
[centers,radii] = imfindcircles(imMask,[6 12],'ObjectPolarity','dark','Sensitivity',0.93);
[imagePoints, boardSize] = detectCheckerboardPoints(im);
imagePoints = imagePoints + newOrigin; 
[R, T] = extrinsics(imagePoints, worldPoints, cameraParams);
centers2 = centers + newOrigin;
centers2 = double(centers2);
centerPlate_stud_image = 0.25* [(centers2(1)+centers2(2)+centers2(3)+centers2(4)), (centers2(5)+centers2(6)+centers2(7)+centers2(8))];
centerPlate_stud_world = pointsToWorld(cameraParams, R, T, centerPlate_stud_image);
centerPlate_stud_world = [centerPlate_stud_world 0];
distanceToCenter_studs = norm(centerPlate_stud_world);
fprintf('Distance from the reference origin to the build plate centre = %0.2f mm\n', ...
    distanceToCenter_studs);

circles = [centers, radii];
im = insertObjectAnnotation(im, 'circle', circles, 'AP CORNER');
imshow(im, 'InitialMagnification', magnification);
hold on
h = viscircles(centers,radii);
hold on
plot(centerPlate_stud_image(1)-newOrigin(1),centerPlate_stud_image(2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'red', 'linewidth', 2);
hold on
plot(imagePoints(1,1)-newOrigin(1),imagePoints(1,2)-newOrigin(2),'*', 'MarkerSize', 20, 'MarkerEdgeColor', 'blue','linewidth', 2);

[~, idx]=sort(centers2(:,1),'Ascend');
theta_A = atan((centers2(idx(1),1)-centers2(idx(2),1))/(centers2(idx(1),2)-centers2(idx(2),2)));
theta_B = atan((centers2(idx(3),1)-centers2(idx(4),1))/(centers2(idx(3),2)-centers2(idx(4),2)));
if le(theta_A*theta_B,0)
   theta_avg = 0;
else 
    theta_avg = 0.5*(theta_A+theta_B);
end
theta_avg_deg = theta_avg*180/pi;
fprintf('Angle of assembly plate relative to reference pattern= %0.2f deg\n', ...
    theta_avg_deg);
% SPI 
vertices = [(imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160);...
    (imagePoints(1,1)-newOrigin(1)+330) (imagePoints(1,2)-newOrigin(2)-230);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-350);...
    (imagePoints(1,1)-newOrigin(1)+560) (imagePoints(1,2)-newOrigin(2)-320);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-150);...
    (imagePoints(1,1)-newOrigin(1)+260) (imagePoints(1,2)-newOrigin(2)-80);...
    (imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160)];

poly = drawpolygon('Position',vertices);

maskSPI = createMask(poly, imHSV);
imLab = rgb2lab(im);
delete(poly);

%red and green
imMask_A = imLab(:,:,2) .* maskSPI;
t_A = graythresh(imMask_A);
imMask_A_red = (imMask_A>t_A+10);
imMask_A_green = (imMask_A<t_A-10);
%beige
imMask_L = imLab(:,:,1) .* maskSPI;
imMask_B = imLab(:,:,3) .* maskSPI;
max_L = max(imMask_L,[],'all');
max_B = max(imMask_B,[],'all');
t_L = 0.67*max_L;
t_B = 0.54*max_B;
imMask_L_bright = (imMask_L>t_L);
imMask_B_yellow = (imMask_B>t_B);
imMask_beige=imMask_L_bright;
imMask_beige(imMask_B_yellow==0)=0;
%white
t_L2 = 0.8*max_L;
imMask_white = (imMask_L>t_L2);
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
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
     boxes_red, 'Red 2x3 Brick');
[areas_green, boxes_green] = step(blobAnalysis, imMask_A_green);
% Sort connected components in descending order by area
[~, idx_green] = sort(areas_green, 'Descend');
% Get the n largest components.
boxes_green = double(boxes_green(idx_green(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
    boxes_green, 'Green 2x6 Plate');
[areas_beige, boxes_beige] = step(blobAnalysis, imMask_beige);
% Sort connected components in descending order by area
[~, idx_beige] = sort(areas_beige, 'Descend');
% Get the n largest components.
boxes_beige = double(boxes_beige(idx_beige(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
    boxes_beige, 'Beige 2x10 Plate');
[areas_white, boxes_white] = step(blobAnalysis, imMask_white);
% Sort connected components in descending order by area
[~, idx_white] = sort(areas_white, 'Descend');
% Get the n largest components.
boxes_white = double(boxes_white(idx_white(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
   boxes_white, 'White 2x6 Brick');

imshow(im);
hold on
h = viscircles(centers,radii);
hold on
plot(centerPlate_stud_image(1)-newOrigin(1),centerPlate_stud_image(2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'red', 'linewidth', 2);
hold on
plot(imagePoints(1,1)-newOrigin(1),imagePoints(1,2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'blue','linewidth', 2);

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

%% Frame 2
[im, newOrigin] = undistortImage(frame2, cameraParams, 'OutputView', 'full');

figure;
imshow(im, 'InitialMagnification', magnification);
imHSV = rgb2hsv(im);
% figure;imshow(imHSV(:,:,3),'InitialMagnification', magnification)
rectsize = [400 320];
rect = drawrectangle('Position',[(imagePoints(1,1)-newOrigin(1)-0.5*(rectsize(1))),imagePoints(1,2)-newOrigin(2)-1.1*(rectsize(2)),rectsize(1),rectsize(2)]);

% SPI 
vertices = [(imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160);...
    (imagePoints(1,1)-newOrigin(1)+330) (imagePoints(1,2)-newOrigin(2)-230);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-350);...
    (imagePoints(1,1)-newOrigin(1)+560) (imagePoints(1,2)-newOrigin(2)-320);...
    (imagePoints(1,1)-newOrigin(1)+420) (imagePoints(1,2)-newOrigin(2)-150);...
    (imagePoints(1,1)-newOrigin(1)+260) (imagePoints(1,2)-newOrigin(2)-80);...
    (imagePoints(1,1)-newOrigin(1)+220) (imagePoints(1,2)-newOrigin(2)-160)];

poly = drawpolygon('Position',vertices);

maskSPI = createMask(poly, imHSV);
imLab = rgb2lab(im);
delete(poly);

%red and green
imMask_A = imLab(:,:,2) .* maskSPI;
t_A = graythresh(imMask_A);
imMask_A_red = (imMask_A>t_A+10);
imMask_A_green = (imMask_A<t_A-10);
%beige
imMask_L = imLab(:,:,1) .* maskSPI;
imMask_B = imLab(:,:,3) .* maskSPI;
max_L = max(imMask_L,[],'all');
max_B = max(imMask_B,[],'all');
t_L = 0.67*max_L;
t_B = 0.54*max_B;
imMask_L_bright = (imMask_L>t_L);
imMask_B_yellow = (imMask_B>t_B);
imMask_beige=imMask_L_bright;
imMask_beige(imMask_B_yellow==0)=0;
%white
t_L2 = 0.8*max_L;
imMask_white = (imMask_L>t_L2);
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
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
     boxes_red, 'Red 2x3 Brick');
[areas_green, boxes_green] = step(blobAnalysis, imMask_A_green);
% Sort connected components in descending order by area
[~, idx_green] = sort(areas_green, 'Descend');
% Get the n largest components.
boxes_green = double(boxes_green(idx_green(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
    boxes_green, 'Green 2x6 Plate');
[areas_beige, boxes_beige] = step(blobAnalysis, imMask_beige);
% Sort connected components in descending order by area
[~, idx_beige] = sort(areas_beige, 'Descend');
% Get the n largest components.
boxes_beige = double(boxes_beige(idx_beige(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
    boxes_beige, 'Beige 2x10 Plate');
[areas_white, boxes_white] = step(blobAnalysis, imMask_white);
% Sort connected components in descending order by area
[~, idx_white] = sort(areas_white, 'Descend');
% Get the n largest components.
boxes_white = double(boxes_white(idx_white(1), :));
% Insert labels for the Legos.
im = insertObjectAnnotation(im, 'rectangle', ...
   boxes_white, 'White 2x6 Brick');

imshow(im, 'InitialMagnification', magnification);

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
%% Frame 3
[im, newOrigin] = undistortImage(frame3, cameraParams, 'OutputView', 'full');
figure;
imshow(im, 'InitialMagnification', magnification);

%AP
imHSV = rgb2hsv(im);
% figure;imshow(imHSV(:,:,3),'InitialMagnification', magnification)
rectsize = [390 320];
rect = drawrectangle('Position',[(imagePoints(1,1)-newOrigin(1)-0.5*(rectsize(1))),imagePoints(1,2)-newOrigin(2)-1.1*(rectsize(2)),rectsize(1),rectsize(2)]);

mask = createMask(rect,imHSV);
imMask = imHSV(:,:,3) .* mask;
[centers,radii] = imfindcircles(imMask,[6 12],'ObjectPolarity','dark','Sensitivity',0.93);
[imagePoints, boardSize] = detectCheckerboardPoints(im);
imagePoints = imagePoints + newOrigin; 
[R, T] = extrinsics(imagePoints, worldPoints, cameraParams);
centers2 = centers + newOrigin;
centers2 = double(centers2);
centerPlate_stud_image = 0.25* [(centers2(1)+centers2(2)+centers2(3)+centers2(4)), (centers2(5)+centers2(6)+centers2(7)+centers2(8))];
centerPlate_stud_world = pointsToWorld(cameraParams, R, T, centerPlate_stud_image);
centerPlate_stud_world = [centerPlate_stud_world 0];
distanceToCenter_studs = norm(centerPlate_stud_world);
fprintf('Distance from the reference origin to the build plate centre = %0.2f mm\n', ...
    distanceToCenter_studs);

circles = [centers, radii];
im = insertObjectAnnotation(im, 'circle', circles, 'AP CORNER');
imshow(im, 'InitialMagnification', magnification);
hold on
h = viscircles(centers,radii);
hold on
plot(centerPlate_stud_image(1)-newOrigin(1),centerPlate_stud_image(2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'red', 'linewidth', 2);
hold on
plot(imagePoints(1,1)-newOrigin(1),imagePoints(1,2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'blue','linewidth', 2);

[~, idx]=sort(centers2(:,1),'Ascend');
theta_A = atan((centers2(idx(1),1)-centers2(idx(2),1))/(centers2(idx(1),2)-centers2(idx(2),2)));
theta_B = atan((centers2(idx(3),1)-centers2(idx(4),1))/(centers2(idx(3),2)-centers2(idx(4),2)));
if le(theta_A*theta_B,0)
   theta_avg = 0;
else 
    theta_avg = 0.5*(theta_A+theta_B);
end
theta_avg_deg = theta_avg*180/pi;
fprintf('Angle of assembly plate relative to reference pattern= %0.2f deg\n', ...
    theta_avg_deg);