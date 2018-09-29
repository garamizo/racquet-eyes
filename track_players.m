%% Estimate background

vsrc = VideoReader('data/videos/20170201_120045.MOV');
vsrc.CurrentTime = 20;

frame = vsrc.readFrame();
frames = zeros([size(frame), 50], 'uint8');
skipTime = (vsrc.Duration - vsrc.CurrentTime) / 100;

for k = 1 : size(frames, 4)
    frames(:,:,:,k) = vsrc.readFrame();
    vsrc.CurrentTime = vsrc.CurrentTime + skipTime;
    if mod(k, 10) == 0
        fprintf('Completed %d out of %d...\n', k, size(frames, 4))
    end
end
fprintf('Done saving frames\n')

bg = median(frames, 4);
figure, imshow(bg)

% show frames
figure
for k = 1 : size(frames, 4)
    imshow(frames(:,:,:,k))
    pause(0.15)
end

%% find MAE threshold to call foreground
bg_double = cast(bg, 'double');

err = zeros(numel(bg(:,:,1)), size(frames, 4));
for k = 1 : size(frames, 4)
    tmp = sum(abs(cast(frames(:,:,:,k), 'double') - bg_double), 3);
    err(:,k) = tmp(:);
end

figure, histogram(err)

%%
% clear
load('extrinsic_calibration.mat')

% vsrc = VideoReader('data/videos/20170201_120045.MOV');
vsrc.CurrentTime = 300;
skipFrames = 5;

ylikely = @(x, y) ParticleFilter.ImageMeasurementLikelihoodFcn(x, y, cameraParams, ...
                                          rotationMatrix, translationVector);

pfm = particleFilter(@(x) ParticleFilter.ImageUpdateEq(x, skipFrames/vsrc.FrameRate), ...
    ylikely);
% pfm.ResamplingPolicy.TriggerMethod = 'interval';
% pfm.ResamplingPolicy.SamplingInterval = 3;
pfm.ResamplingPolicy.MinEffectiveParticleRatio = 0.6;
pfm.ResamplingMethod = 'multinomial';
initialize(pfm, 50, [-10, 10; 0, 40], 'StateOrientation', 'row');

bg_double = cast(bg, 'double');

nbest = 20;  % number of candidate tracks
colors = colormap('hsv');
colors = colors(round(linspace(1, size(colors,1), nbest)),:);
frame  = vsrc.readFrame();

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

clear h1 h2 h3 h4
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
for k = 1 : nbest
    h4(k) = plot(0, 0, 'ko', 'linewidth', 5, 'markersize', 50);
    h1(k) = plot(0, 0, 'x', 'linewidth', 5, 'markersize', 20, 'color', colors(k,:));
    
end
axis equal, xlim([-10, 10]), ylim([0, 40]), grid on, xlabel('X_W'), ylabel('Y_W')

subplot(122), hi = imshow(frame);
hold on,
for k = 1 : nbest
    h2(k) = fill([0, 0], [0, 0], colors(k,:), 'linewidth', 1, 'FaceAlpha', 0.02, 'EdgeColor', colors(k,:));
    h3(k) = plot(0, 0, 'o', 'linewidth', 3, 'markersize', 20, 'color', colors(k,:));
end
set(gcf, 'Position', [1          41        1680         933])

%%
while hasFrame(vsrc)
    %
    tic
    frame  = vsrc.readFrame();
    vsrc.CurrentTime = vsrc.CurrentTime + (skipFrames - 1) / vsrc.FrameRate;
    
    mask = sum(abs(cast(frame, 'double') - bg_double), 3) > 150;
    
    [~, Px] = pfm.correct(mask);

    X = pfm.Particles;
    P = pfm.Weights;
    
    [Psort, idx] = sort(P, 'descend');
%     ylikely(X(idx(1:10),:), mask)
    

    
    set(hi, 'CData', frame)
    for p = 1 : nbest
        footprint = [X(idx(p), 1:2), 0];  % in ft

        w = 0.75;  % width/2
        h = 6.0;
        bf = h * 0.0;  % bottom offset
        tf = h/6;  % top offset
        box = ([w, 0, bf    
               w, 0, h-tf
               -w, 0, h-tf
               -w, 0, bf
               w, 0, bf ] + footprint) * 304.8;

        iPts = worldToImage(cameraParams,rotationMatrix,translationVector, box);
        iPts_foot = worldToImage(cameraParams,rotationMatrix,translationVector, footprint * 304.8);

        set(h1(p), 'XData', footprint(:,1), 'YData', footprint(:,2))
        set(h2(p), 'XData', iPts(:,1), 'YData', iPts(:,2))
        set(h3(p), 'XData', iPts_foot(1), 'YData', iPts_foot(2))
    end
    
%     tic
%     gm = fitgmdist(X,2);
%     toc
%     for p = 1 : 2
%         set(h4(p), 'XData', gm.mu(p,1), 'YData', gm.mu(p,2))
%     end
    
    pfm.predict();

    pause(skipFrames/vsrc.FrameRate - toc)
end
