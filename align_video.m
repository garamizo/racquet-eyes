%% Track players with normalized 2-d cross-correlation

%% Calibrate camera

vfile = 'videos/calib2.mov';

% manually select frames for calibration
implay(vfile);

frameid = [1211 1340 1560 1710 1790 2220 2300 3180 3360 3490 3600 3810, ...
    4620 4770 4920 5160 5220 5370 5490 5610  5760 5820 5910 6030];
  
vid = VideoReader(vfile);

clear img
for k = 1 : length(frameid)
    img = rgb2gray(read(vid, frameid(k)));
    imwrite(img, fullfile('images\calibration2', sprintf('img%02d.jpg', k)))
end

% launch Camera Calibrator app and export cameraParams variable

%% Find background

load('cameraParams_fisheye', 'cameraParams');
vfile = fullfile('videos', '20170917_182446.MOV');

vid = VideoReader(vfile);
duration = [14, 2*60+15];

vid.CurrentTime = duration(1);
dt = 1;

% sat = zeros(vid.Height, vid.Width, length(idx));
img0 = uint8(zeros(vid.Height, vid.Width, 3, ceil(diff(duration) / dt)));
for k = 1 : size(img0, 4)
%     I = read(vid, idx(k));
%     Irect = undistortImage(I, cameraParams);
%     hsv  = rgb2hsv(Irect);
%     sat(:,:,k) = squeeze(hsv(:,:,2));
    img0(:,:,:,k) = readFrame(vid);
    vid.CurrentTime = vid.CurrentTime + dt;
end
% sat0 = median(sat, 3);
backg = undistortImage(median(img0, 4), cameraParams);
hsv = rgb2hsv(backg);
sat0 = squeeze(hsv(:,:,2));

imshow(sat0)

%% calculate extrinsics

BW = edge(sat0, 'canny', 0.3);

[H, T, R] = hough(BW, 'RhoResolution', 1, 'ThetaResolution', 0.1);

P  = houghpeaks(H, 5, 'threshold', 0); % sorted by line strength
lines = houghlines(BW, T, R, P, 'FillGap', 100, 'MinLength', 500);

[~, hidx] = sort(abs([lines.theta]), 'descend'); % get horizontal lines
[~, ridx] = sort(abs([lines(hidx(1:3)).rho]));
hidx(ridx); % back, short, server line

sidx = setdiff(1:5, hidx(ridx)); % side lines
[~, tidx] = sort([lines(sidx).theta]);
sidx(tidx); % backhand, forehand line

% back, short, server, backhand, forehand
lines = lines([hidx(ridx), sidx(tidx)]);

% figure, imshow(BW)
% 
% figure
% imshow(imadjust(mat2gray(H)), 'XData', T, 'Ydata', R, 'InitialMagnification', 'fit')
% title('Hough transform of gantrycrane.png');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% colormap(gca,hot);
% 
% figure, imshow(Irect), hold on
% for kl = 1 : length(P)
%    xy = [lines(kl).point1; lines(kl).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% end

% back, short, server, backhand, forehand
th = [lines.theta]' * pi/180;
r = [lines.rho]';

imagePoints = [];
for k2 = 4 : 5
    for k1 = 1 : 3
        alf = [sin(th(k1)), -sin(th(k2)); -cos(th(k1)), cos(th(k2))] \ ...
            [-r(k1)*cos(th(k1)) + r(k2)*cos(th(k2))
            -r(k1)*sin(th(k1)) + r(k2)*sin(th(k2))];
        pint = r(k1)*[cos(th(k1)), sin(th(k1))] + alf(1)*[sin(th(k1)), -cos(th(k1))];
        imagePoints = [imagePoints; pint];
    end
end

worldPoints = [-10, 0, 0
    -10, 20, 0
    -10, 25, 0
    10, 0, 0
    10, 20, 0
    10, 25, 0];

imagePoints = [    1.1339    0.4312
    1.3664    0.6313
    1.5003    0.7476
    0.5850    0.4001
    0.3187    0.5826
    0.1551    0.6948] * 1e3;

[rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints(:,1:2), cameraParams);
% [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams)
rotationMatrix = [-0.9974   -0.0479   -0.0539
    0.0355    0.3247   -0.9452
    0.0628   -0.9446   -0.3221];
translationVector = [-1.9156   -6.0412   39.3986];

camMatrix = cameraMatrix(cameraParams, rotationMatrix, translationVector);

origin = [worldPoints, ones(size(worldPoints,1), 1)] * camMatrix;
origin = origin(:,1:2) ./ origin(:,3);

figure, imshow(Irect), hold on
plot(imagePoints(:,1), imagePoints(:,2), 'o', 'markersize', 5, 'linewidth', 2)
text(imagePoints(:,1), imagePoints(:,2), num2str((1:6)'), ...
    'color', [1 1 1], 'fontsize', 20)

plot(origin(:,1), origin(:,2), 's', 'linewidth', 2)

%% rotate and crop

wid = size(Irect, 2);
hei = size(Irect, 1);

q = mean(atan2(imagePoints(1:3,2) - imagePoints(4:6,2), ...
    imagePoints(1:3,1) - imagePoints(4:6,1))) * 180/pi;
imorigin = mean(imagePoints([1 4],:));
imoriginr = [imorigin - [wid/2, hei/2], 0] * Rz(q * pi/180);
imoriginr = round(imoriginr(1:2) + [wid/2, hei/2]);

img = imrotate(Irect, q, 'bilinear', 'crop');
img = img((-340:580) + imoriginr(2), (-800:800) + imoriginr(1),:);

imshow(img), hold on
plot(imoriginr(1), imoriginr(2), 'o')

%% foreground selection

vid.CurrentTime = duration(1);
Irect = undistortImage(readFrame(vid), cameraParams);

hsv2 = rgb2hsv(Irect);
hsv1 = rgb2hsv(backg);

mask = abs(hsv1(:,:,3) - hsv2(:,:,3)) > 0.05 & ...
    abs(hsv1(:,:,1) - hsv2(:,:,1)) > 0.005;

figure
subplot(321), imagesc(hsv2(:,:,1)), colorbar
subplot(323), imagesc(hsv2(:,:,2)), colorbar
subplot(325), imagesc(hsv2(:,:,3)), colorbar
subplot(322), imagesc(hsv1(:,:,1)), colorbar
subplot(324), imagesc(hsv1(:,:,2)), colorbar
subplot(326), imagesc(hsv1(:,:,3)), colorbar

figure, imshow(mask)

%%
    

for k = 1000 : 50 : nframes
    tic
    I = read(vid, k);
    Irect = undistortImage(I, cameraParams);
    hold off
    imshow(Irect)
    hold on
    hsv  = rgb2hsv(Irect);
    BW = edge(squeeze(hsv(:,:,2)),'canny', 0.5, 5);
    
    [H,T,R] = hough(BW);
    P  = houghpeaks(H,15,'threshold',ceil(0.3*max(H(:))));
    
    lines = houghlines(BW,T,R,P,'FillGap',300,'MinLength',300);
    
    for kl = 1:length(lines)
       xy = [lines(kl).point1; lines(kl).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    end
    drawnow
    pause(0.1 - toc)
end

