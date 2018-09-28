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
vsrc.CurrentTime = 45;
skipFrames = 5;

ylikely = @(x, y) ParticleFilter.ImageMeasurementLikelihoodFcn(x, y, cameraParams, ...
                                          rotationMatrix, translationVector);

pfm = particleFilter(@(x) ParticleFilter.ImageUpdateEq(x, skipFrames/vsrc.FrameRate), ...
    ylikely);
pfm.ResamplingPolicy.TriggerMethod = 'interval';
pfm.ResamplingPolicy.SamplingInterval = 3;
% pfm.ResamplingPolicy.MinEffectiveParticleRatio = 0.6;
pfm.ResamplingMethod = 'multinomial';
initialize(pfm, 100, [-10, 10; 0, 40], 'StateOrientation', 'row');

bg_double = cast(bg, 'double');

nbest = 100;  % number of candidate tracks
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

clear h1 h2
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
for k = 1 : nbest
    h1(k) = plot(0, 0, 'x', 'linewidth', 5, 'markersize', 20, 'color', colors(k,:));
end
axis equal, xlim([-10, 10]), ylim([0, 40]), grid on, xlabel('X_W'), ylabel('Y_W')

subplot(122), h = imshow(frame);
hold on,
for k = 1 : nbest
    h2(k) = fill([0, 0], [0, 0], colors(k,:), 'linewidth', 1, 'FaceAlpha', 0.3, 'EdgeColor', colors(k,:));
end
set(gcf, 'Position', [1          41        1680         933])


while hasFrame(vsrc)
    %%
    tic
    frame  = vsrc.readFrame();
    vsrc.CurrentTime = vsrc.CurrentTime + (skipFrames - 1) / vsrc.FrameRate;
    
    mask = sum(abs(cast(frame, 'double') - bg_double), 3) > 150;
    
    [~, Px] = pfm.correct(mask);

    X = pfm.Particles;
    P = pfm.Weights;
    
    [Psort, idx] = sort(P, 'descend');
    
    set(h, 'CData', frame)
    for p = 1 : nbest
        footprint = [X(idx(p), 1:2), 0];  % in ft
        height = 6.5;

        wid = 2;
        box = ([wid/2, 0, 0
               wid/2, 0, height
               -wid/2, 0, height
               -wid/2, 0, 0
               wid/2, 0, 0 ] + footprint) * 304.8;

        iPts = worldToImage(cameraParams,rotationMatrix,translationVector, box);

        set(h1(p), 'XData', footprint(:,1), 'YData', footprint(:,2))
        set(h2(p), 'XData', iPts(:,1), 'YData', iPts(:,2))
    end
    
    pfm.predict();

    pause(skipFrames/vsrc.FrameRate - toc)
end
