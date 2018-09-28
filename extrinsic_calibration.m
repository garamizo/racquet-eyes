%% Select frame for extrinsic calibration
%   (Do it only once. Save an image file)
clear

filename = 'data/20170917_182446.MOV';
v = VideoReader(filename);
v.CurrentTime = 20;

dt = 1 / 30;
figure, h = imshow(rgb2gray(readFrame(v)));

while hasFrame(v)
    tic
    frame = rgb2gray(readFrame(v));
    set(h, 'CData', frame)
    pause(dt - toc)
end

imwrite(frame, 'data/extrinsic.jpg')

%% Select image points by hand

frame = imread('data/extrinsic.jpg');
figure, imshow(frame)
[x, y] = ginput();

imagePoints = [x, y];

% camera on back position
worldPoints = [10, 15, 0
                10, 20, 0
                10, 25, 0
                10, 40, 0
                -10, 40, 0
                -10, 25, 0
                -10, 20, 0
                -10, 15, 0 ] * 304.8;
            
% camera on front position
% worldPoints = [-10, 25, 0
%                 -10, 20, 0
%                 -10, 15, 0
%                 -10, 0, 0
%                 10, 0, 0
%                 10, 15, 0
%                 10, 20, 0
%                 10, 25, 0 ] * 304.8;

%% Extrinsic calibration
load('cameraParams.mat', 'cameraParams');

[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams, ...
    'MaxNumTrials', 10000);
% worldOrientation maps world to camera coordinates
% worldLocation is in world coordinates

rotationMatrix = worldOrientation';
translationVector = -worldLocation * worldOrientation';

% test on imagePoints
worldPointsEst = pointsToWorld(cameraParams,rotationMatrix,translationVector,imagePoints);
error = sqrt(sum((worldPointsEst - worldPoints(:,1:2)).^2, 2))

%% Animation
%   Click points on right subplot to see the projection on the court

clear
load('extrinsic_calibration.mat')

outCourt = [10, 0
            10, 40
            -10, 40
            -10, 0
            10, 0 ];
        
serveBox = [10, 20
            10, 25
            -10, 25
            -10, 20
            10, 20
            NaN, NaN
            8.5, 20
            8.5, 25
            7, 25
            7, 20
            8.5, 20
            NaN, NaN
            -8.5, 20
            -8.5, 25
            -7, 25
            -7, 20
            -8.5, 20];
        
shortLine = [10, 15
            -10, 15];
        
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
axis equal, xlim([-10, 10]), ylim([0, 40]), grid on, xlabel('X_W'), ylabel('Y_W')
subplot(122), imshow(frame)
set(gcf, 'Position', [0.0010    0.0410    1.5360    0.7488]*1e3)

[x, y] = getpts();
subplot(122), hold on, plot(x, y, 'x', 'linewidth', 5, 'markersize', 15)
wPts = pointsToWorld(cameraParams,rotationMatrix,translationVector,[x, y]);
wPts_ft = wPts / 304.8;
subplot(121), plot(wPts_ft(:,1), wPts_ft(:,2), 'x', 'linewidth', 5, 'markersize', 20)

%% From 3D to image

% footprint = [-4, 3, 0];  % in ft, Leslie
% height = 6;

footprint = [4, 3, 0];  % in ft, Gui
height = 5.5;

% footprint = [7.75, 22.5, 0];  % in ft, Gui
% height = 5.5;

box = ([1, 0, 0
        1, 0, height/2
       1, 0, height
       -1, 0, height
       -1, 0, height/2
       -1, 0, 0
       1, 0, 0 ] + footprint) * 304.8;

iPts = worldToImage(cameraParams,rotationMatrix,translationVector, box);
        
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
plot(footprint(:,1), footprint(:,2), 'x', 'linewidth', 5, 'markersize', 20)
axis equal, xlim([-10, 10]), ylim([0, 40]), grid on, xlabel('X_W'), ylabel('Y_W')
subplot(122), imshow(frame)
hold on, fill(iPts(:,1), iPts(:,2), [0.7, 0, 0], 'linewidth', 1, 'FaceAlpha', 0.3, 'EdgeColor', [0.7, 0, 0])
set(gcf, 'Position', [0.0010    0.0410    1.5360    0.7488]*1e3)
