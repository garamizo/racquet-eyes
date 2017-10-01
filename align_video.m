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

vfile = 'videos/20170917_182446.mov';
vid = VideoReader(vfile);
nframes = round(vid.Duration * vid.FrameRate);

sat = zeros(vid.Height, vid.Width, 20);
idx = 1000 : round(nframes/20) : nframes;
for k = 1 : length(idx)
    I = read(vid, idx(k));
    Irect = undistortImage(I, cameraParams);
    hsv  = rgb2hsv(Irect);
    sat(:,:,k) = squeeze(hsv(:,:,2));
end
sat0 = median(sat, 3);

%% Align video

BW = edge(sat0, 'canny', 0.5, 5);
[H, T, R] = hough(BW);

P  = houghpeaks(H, 3, 'threshold', 0); % sorted by line strength
% expected values: psi+eta and -psi+eta
eta = ( T(P(1,2)) + T(P(2,2)) )/2;


% P  = houghpeaks(H, 15, 'threshold', ceil(0.3*max(H(:))));
lines = houghlines(BW, T, R, P, 'FillGap', 300, 'MinLength', 300);

figure, imshow(Irect), hold on
for kl = 1:length(lines)
   xy = [lines(kl).point1; lines(kl).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end

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

