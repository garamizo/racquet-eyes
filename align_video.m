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

load('cam_calib_params', 'cameraParams');

vfile = 'videos/20170917_182446.mov';

vid = VideoReader(vfile);
frames = round([14, 2*60+15] * vid.FrameRate);

idx = frames(1) : round(diff(frames)/30) : frames(2);
sat = zeros(vid.Height, vid.Width, length(idx));
for k = 1 : length(idx)
    I = read(vid, idx(k));
    Irect = undistortImage(I, cameraParams);
    hsv  = rgb2hsv(Irect);
    sat(:,:,k) = squeeze(hsv(:,:,2));
end
sat0 = median(sat, 3);

%% Align video

BW = edge(sat0, 'canny', 0.3);
BW = imrotate(BW, 0);
% BW = BW(100:end-100,100:end-100);
imshow(BW)
%%

[H, T, R] = hough(BW, 'RhoResolution', 1, 'ThetaResolution', 0.1);

figure
imshow(imadjust(mat2gray(H)), 'XData', T, 'Ydata', R, 'InitialMagnification', 'fit')
title('Hough transform of gantrycrane.png');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);
%%

P  = houghpeaks(H, 5, 'threshold', 0); % sorted by line strength
% expected values: psi+eta and -psi+eta
% [~, idx] = min( abs(abs(T(P(:,2))) - 90) );
% eta = ( T(P(1,2)) + T(P(2,2)) )/2;

% P  = houghpeaks(H, 15, 'threshold', ceil(0.3*max(H(:))));
lines = houghlines(BW, T, R, P, 'FillGap', 100, 'MinLength', 500);

[~, hidx] = sort(abs([lines.theta]), 'descend'); % get horizontal lines
[~, ridx] = sort(abs([lines(hidx(1:3)).rho]));
hidx(ridx); % back, short, server line

sidx = setdiff(1:5, hidx(ridx)); % side lines
[~, tidx] = sort([lines(sidx).theta]);
sidx(tidx); % backhand, forehand line

% back, short, server, backhand, forehand
lines = lines([hidx(ridx), sidx(tidx)]);

figure, imshow(Irect), hold on
for kl = 1 : length(P)
   xy = [lines(kl).point1; lines(kl).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end

%% calculate points

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

figure, imshow(I), hold on
plot(imagePoints(:,1), imagePoints(:,2), 'o', 'markersize', 5, 'linewidth', 2)
text(imagePoints(:,1), imagePoints(:,2), num2str((1:6)'), ...
    'color', [1 1 1], 'fontsize', 20)

worldPoints = [-10, 0
    -10, 20
    -10, 25
    10, 0
    10, 20
    10, 25];

[rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams);
translationVector(3) = 39.5;
camMatrix = cameraMatrix(cameraParams, rotationMatrix, translationVector);

origin = [  10,     0,     0, 1
            -10,    0,     0, 1
            10,     20,     0, 1
            -10,    20,     0, 1
            10,     25,     0, 1
            -10,    25,     0, 1] * camMatrix;
origin = origin(:,1:2) ./ origin(:,3);

plot(origin(:,1), origin(:,2), 's')

%%

T = -translationVector * rotationMatrix';
R = rotationMatrix;

clear q
[q(1), q(2), q(3)] = dcm2angle(R, 'YZX');

img = imrotate(Irect, -q(2)*180/pi);
imshow(img)


%%
syms pitch z i j

t = sym('t', [3, 1]);
P = Ry(sym(pi)) * Rx(pitch) * [j*z/f; i*z/f; z] + t;


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

