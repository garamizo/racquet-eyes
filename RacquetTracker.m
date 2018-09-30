classdef RacquetTracker < handle

    properties
        startTime = 160
        skipFrames = 5
        update_dt  % at every frame

        calibFile = 'extrinsic_calibration.mat'
        videoFile = 'data/videos/20170201_120045.MOV'
        
        backtresh = 150
        
        boxWid = 1.0
        boxHeight = 6.0
        boxBound = 1.0
        
        speed_std = 15  % feet/s
        numParticles = 100
        showParticles = 10
    end
    
    properties (Access = private)
        vsrc
        pf
        
        box
        outbox
        backg

        cameraParams
        rotationMatrix
        translationVector
        
        % graphics
        outCourt, serveBox, shortLine
        colors
        
        % plot handles
        hi, h1, h2, h3
    end
    
    methods
        function obj = RacquetTracker()

            % setup video
            obj.vsrc = VideoReader(obj.videoFile);
            obj.vsrc.CurrentTime = obj.startTime;
            
            obj.update_dt = obj.skipFrames / obj.vsrc.FrameRate;
            
            % setup particle filter
            obj.pf = particleFilter(@(x) obj.UpdateFcn(x), ...
                @(x, y) obj.MeasurementLikelihoodFcn(x, y));
            obj.pf.ResamplingPolicy.TriggerMethod = 'interval';
            obj.pf.ResamplingPolicy.SamplingInterval = 3;
%             obj.pf.ResamplingPolicy.MinEffectiveParticleRatio = 0.2;
            obj.pf.ResamplingMethod = 'multinomial';
            initialize(obj.pf, obj.numParticles, [-10, 10; 0, 40], 'StateOrientation', 'row');

            obj.box = [ obj.boxWid, 0, 0
                        obj.boxWid, 0, obj.boxHeight
                        -obj.boxWid, 0, obj.boxHeight
                        -obj.boxWid, 0, 0
                        obj.boxWid, 0, 0 ];
                    
            obj.outbox = [  obj.boxWid+obj.boxBound, 0, -obj.boxBound
                            obj.boxWid+obj.boxBound, 0, obj.boxHeight+obj.boxBound
                            -obj.boxWid-obj.boxBound, 0, obj.boxHeight+obj.boxBound
                            -obj.boxWid-obj.boxBound, 0, -obj.boxBound
                            obj.boxWid+obj.boxBound, 0, -obj.boxBound
                            obj.boxWid, 0, 0
                            obj.boxWid, 0, obj.boxHeight
                            -obj.boxWid, 0, obj.boxHeight
                            -obj.boxWid, 0, 0
                            obj.boxWid, 0, 0 ];
                        
            load(obj.calibFile, 'cameraParams', 'rotationMatrix', 'translationVector', 'bg');
            obj.cameraParams = cameraParams;
            obj.rotationMatrix = rotationMatrix;
            obj.translationVector = translationVector;
            obj.backg = cast(bg, 'double');
            
            obj.outCourt = [10, 0
                        10, 40
                        -10, 40
                        -10, 0
                        10, 0 ];

            obj.serveBox = [10, 20
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

            obj.shortLine = [10, 15
                             -10, 15];
                    
            obj.colors = colormap('hsv');
            obj.colors = obj.colors(round(linspace(1, size(obj.colors,1), obj.numParticles)),:);
            obj.colors = obj.colors(randperm(obj.numParticles),:);
        end
        
        function [frame, mask] = readFrame(obj)
            frame = obj.vsrc.readFrame();
            mask = sum(abs(cast(frame, 'double') - obj.backg), 3) > obj.backtresh;
            obj.vsrc.CurrentTime = obj.vsrc.CurrentTime + obj.skipFrames / obj.vsrc.FrameRate;
        end
        
        function step(obj)
            [frame, mask]  = obj.readFrame();
            obj.pf.correct(mask);
            [~, idx] = sort(obj.pf.Weights, 'descend');
            obj.pf.Weights(idx(1:5))'
%             obj.plot(frame, obj.pf.Particles(idx(1:obj.showParticles),:), ...
%                             obj.pf.Weights(idx(1:obj.showParticles)))
            obj.plot(frame, obj.pf.Particles, ...
                            obj.pf.Weights)
            
            obj.pf.predict();
%             rows = randperm(obj.numParticles, 10);
%             obj.pf.Particle(rows,:) = rand(10,2) .* [20, 40] + [-10, 0];
%             obj.pf.Weights(rows,:) = 
        end
        
        function [frame, mask] = lastFrame(obj)
            obj.vsrc.CurrentTime = obj.vsrc.CurrentTime - (obj.skipFrames + 1) / obj.vsrc.FrameRate;
            [frame, mask] = obj.readFrame();
        end
        
        function has_frame = hasFrame(obj)
            has_frame = hasFrame(obj.vsrc);
        end
        
        function Xk = UpdateFcn(obj, X)
            % X: [x, y]
            
            % random velocity
            R = diag([obj.speed_std, obj.speed_std].^2);
            V = mvnrnd([0, 0], R, size(X, 1));

            Xk = X(:,1:2) + V * obj.update_dt;
        end
        
        function P = MeasurementLikelihoodFcn(obj, X, y)
            % y: image
            % X: [x, y] in feet

            P = zeros(size(X, 1), 1);
            for k = 1 : size(X, 1)
                footprint = [X(k,1:2), 0];

                iPts = worldToImage(obj.cameraParams, obj.rotationMatrix, obj.translationVector, ...
                    (obj.box + footprint) * 304.8);
                BW = roipoly(y, iPts(:,1), iPts(:,2));
                
                iPts = worldToImage(obj.cameraParams, obj.rotationMatrix, obj.translationVector, ...
                    (obj.outbox + footprint) * 304.8);
                BWout = roipoly(y, iPts(:,1), iPts(:,2));

                sBW = sum(BW(:));
                sBWout = sum(BWout(:));
                syout = sum(sum(y & BWout, 1), 2);
                sy = sum(sum(y & BW, 1), 2);
                if sBW == 0 || sBWout == 0
                    P(k) = 0;
                else
                    P(k) = sy / sBW - 1 * syout / sBWout;
                    if P(k) < 0
                        P(k) = 0;
                    end
                end
            end
%             P = 1./(1+exp(-P));
            P = (P - min(P)) / (max(P) - min(P));
%             P = P / sum(P);
        end
        
        function [X, P] = image2particle(obj, imagePoint)
            
            [frame, mask] = obj.lastFrame();
            if nargin == 1
                try
                    figure(obj.hi.Parent.Parent)
                catch
                    obj.plot(frame, [], [])
                    figure(obj.hi.Parent.Parent)
                end
                imagePoint = ginput();
            end
                
            X = pointsToWorld(obj.cameraParams, obj.rotationMatrix, ...
                obj.translationVector,imagePoint) / 304.8;
            P = MeasurementLikelihoodFcn(obj, X, mask);
        end
        
        function plot(obj, frame, X, P)
            if nargin == 1
                frame = obj.lastFrame();
                X = obj.pf.Particles;
                P = obj.pf.Weights;
            end
            try
                set(obj.hi, 'CData', frame)
                for p = 1 : size(X, 1)
                    footprint = [X(p,1:2), 0];  % in ft

                    iPts = worldToImage(obj.cameraParams, obj.rotationMatrix,...
                        obj.translationVector, (obj.box + footprint) * 304.8);

                    set(obj.h1(p), 'XData', footprint(:,1), 'YData', footprint(:,2))
                    set(obj.h2(p), 'XData', iPts(:,1), 'YData', iPts(:,2))
                    set(obj.h3(p), 'Position', footprint, 'String', sprintf('%d', round(100*P(p))))
                end
                
                % clear from sight
                for p = size(X, 1) + 1 : obj.numParticles
                    set(obj.h1(p), 'XData', 0, 'YData', -10)
                    set(obj.h2(p), 'XData', [0, 0], 'YData', [0, 0])
                end
            catch
                figure
                plot(obj.outCourt(:,1), obj.outCourt(:,2), 'k', 'linewidth', 5)
                hold on, plot(obj.serveBox(:,1), obj.serveBox(:,2), 'r', 'linewidth', 3)
                plot(obj.shortLine(:,1), obj.shortLine(:,2), 'r--', 'linewidth', 3)
                for k = 1 : obj.numParticles
                    obj.h1(k) = plot(0, -10, 'x', 'linewidth', 5, 'markersize', 20, 'color', obj.colors(k,:));
                    obj.h3(k) = text(0, -10, '');
                end
                axis equal, xlim([-10, 10]), ylim([0, 40]), grid on, xlabel('X_W'), ylabel('Y_W')
                set(gcf, 'Position', [51, 190, 365, 618])

                figure, obj.hi = imshow(frame);
                hold on,
                for k = 1 : obj.numParticles
                    obj.h2(k) = fill([0, 0], [0, 0], obj.colors(k,:), 'linewidth', 1, 'FaceAlpha', 0.02, 'EdgeColor', obj.colors(k,:));
                end
                set(gcf, 'Position', [485, 273, 1184, 662])
                plot(obj, frame, X, P)
            end
        end
    end
    
end

