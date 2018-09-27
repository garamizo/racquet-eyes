%% 
clear

filename = 'data/calibration/asym/20170201_125710.MOV';
v = VideoReader(filename);

minDt = 2;

numSavedImages = 0;
lastFrameTime = 0;
lastFrame = rgb2gray(readFrame(v));
while hasFrame(v)
    frame = rgb2gray(readFrame(v));
       
    if v.CurrentTime - lastFrameTime > minDt
        errorImage = cast(frame, 'double') - cast(lastFrame, 'double');
        mae = mean(abs(errorImage(:)));
        if mae < 50
            numSavedImages = numSavedImages + 1
            lastFrameTime = v.CurrentTime;
            imwrite(frame, sprintf('data/calibration/asym/calib%03d.jpg', numSavedImages));
        end
    end
    
    if numSavedImages > 50
        fprintf('Too many images!\n')
        break
    end
    lastFrame = frame;
end

fprintf('Done!\n')
