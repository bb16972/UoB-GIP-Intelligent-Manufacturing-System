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
imOrig = imread('Image504.png');
figure; imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');
%% Undistort the Image
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');
%% Create rectangular ROI binary mask
imHSV = rgb2hsv(im);
figure;imshow(imHSV(:,:,3),'InitialMagnification', magnification)

rectsize = [400 320];
rect = drawrectangle('Position',[(imagePoints(1,1)-newOrigin(1)-0.5*(rectsize(1))),imagePoints(1,2)-newOrigin(2)-1.1*(rectsize(2)),rectsize(1),rectsize(2)]);

mask = createMask(rect);
% figure;imshow(mask,'InitialMagnification', magnification);

imMask = imHSV(:,:,3) .* mask;
figure;imshow(imMask,'InitialMagnification', magnification);
%% Detect circular studs within ROI
[centers,radii] = imfindcircles(imMask,[6 12],'ObjectPolarity','dark','Sensitivity',0.93);

figure; imshow(imMask, 'InitialMagnification', magnification);
h = viscircles(centers,radii);
title('Detected Studs');
%% Compute Extrinsics
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, T] = extrinsics(imagePoints, worldPoints, cameraParams);
%% Find centre of 4 black studs
centers2 = centers + newOrigin;

centers2 = double(centers2);
centerPlate_stud_image = 0.25* [(centers2(1)+centers2(2)+centers2(3)+centers2(4)), (centers2(5)+centers2(6)+centers2(7)+centers2(8))];

centerPlate_stud_world = pointsToWorld(cameraParams, R, T, centerPlate_stud_image);
centerPlate_stud_world = [centerPlate_stud_world 0];

distanceToCenter_studs = norm(centerPlate_stud_world);
fprintf('Distance from the reference origin to the build plate centre w studs = %0.2f mm\n', ...
    distanceToCenter_studs);

circles = [centers, radii];
im = insertObjectAnnotation(im, 'circle', ...
  circles, 'AP CORNER');

figure; imshow(im, 'InitialMagnification', magnification);
h = viscircles(centers,radii);
hold on
plot(centerPlate_stud_image(1)-newOrigin(1),centerPlate_stud_image(2)-newOrigin(2),'+', 'MarkerSize', 20, 'MarkerEdgeColor', 'red', 'linewidth', 2);
hold on
plot(imagePoints(1,1)-newOrigin(1),imagePoints(1,2)-newOrigin(2),'*', 'MarkerSize', 20, 'MarkerEdgeColor', 'blue','linewidth', 2);
title('Detected Centre');

toc
%% Find angle of assembly plate relative to reference pattern orthogonal axes

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