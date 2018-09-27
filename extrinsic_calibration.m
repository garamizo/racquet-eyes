%% Select frame for extrinsic calibration
%   (Do it only once. Save an image file)
clear

filename = 'data/videos/20170201_120045.MOV';
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

imwrite(frame, 'data/calibration/extrinsic.jpg')

%% Select image points by hand

frame = imread('data/calibration/extrinsic.jpg');
figure, imshow(frame)
[x, y] = ginput();

imagePoints = [x, y];
worldPoints = [10, 15, 0
                10, 20, 0
                10, 25, 0
                10, 40, 0
                -10, 40, 0
                -10, 25, 0
                -10, 20, 0
                -10, 15, 0 ] * 304.8;

%% Extrinsic calibration
load('cameraParams.mat', 'cameraParams');

[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams);
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
set(gcf, 'Position', [1          41        1680         933])

[x, y] = getpts();
subplot(122), hold on, plot(x, y, 'x', 'linewidth', 5, 'markersize', 15)
wPts = pointsToWorld(cameraParams,rotationMatrix,translationVector,[x, y]);
wPts_ft = wPts / 304.8;
subplot(121), plot(wPts_ft(:,1), wPts_ft(:,2), 'x', 'linewidth', 5, 'markersize', 20)

