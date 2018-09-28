classdef ParticleFilter < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    %   Xkk = A*Xk + B*uk + vk
    %   Y = C*Xk + wk
    
    properties
        X  % state particles
        PX
        MeasurementLikelihoodFcn
        updateEq
        num_particles
        
        count
    end
    
    methods
        function obj = ParticleFilter(X0)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.X = X0;
            obj.num_particles = size(X0, 1);
            obj.PX = ones(size(X0,1), 1) / obj.num_particles;
            obj.MeasurementLikelihoodFcn = @ParticleFilter.SampleMeasurementLikelihoodFcn;
            obj.updateEq = @ParticleFilter.SampleUpdateEq;
            
            obj.count = 0;
        end
        
        function [xh, yh] = step(obj, u, y)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % calculate posterior probability
            [P, Yh] = obj.MeasurementLikelihoodFcn(obj.X, y);
%             obj.PX = obj.PX .* (P / sum(P));
            obj.PX = obj.PX .* P;
            obj.PX = obj.PX / sum(obj.PX);
            
            % select best estimates
%             [~, midx] = max(obj.PX);
%             xh = obj.X(midx,:)';
%             yh = Yh(midx,:)';
            xh = sum(obj.X .* obj.PX)' / sum(obj.PX);
            yh = sum(Yh .* obj.PX)' / sum(obj.PX);
            
            % resample states
%             1 / sum(obj.PX.^2)
            obj.count = obj.count + 1;
            if 1 / sum(obj.PX.^2) < obj.num_particles / 3
                obj.count;
                obj.count = 0;
                Ps = cumsum(obj.PX);
                Ps(end) = 1;
                nselect = rand(1, length(Ps)) * Ps(end);
                [~, idx] = min(abs(Ps - nselect));
                
%                 figure, plot(Ps)
%                 hold on, plot(idx, Ps, 'o')
                
                
                obj.X = obj.X(idx,:);
                obj.PX = obj.PX(idx) / sum(obj.PX(idx));
            end
            
            % update state
            obj.X = obj.updateEq(obj.X, u);
            
%             mean(P(idx))
%             mean(P)
        end
    end
    
    methods (Static)
        function Xk = SampleUpdateEq(X, u, W)
            % p(Xkk | uk, Xk)
            if nargin == 2
                R = 1e-5;
                W = mvnrnd(0, R, size(X, 1));
            end
           
            A = [   1, 1
                    0, 1 ];
            B = [   0
                    1 ];
            amp = 50;
            Xk = X * A' + (1/amp)*sin(amp*(u' - W)) * B';
        end
        
        function Yh = SampleMeasurementFcn(X)
            % P(z | X)
            C = [   1, 0
                    0, 1 ];
                
            Yh = X * C';
        end
        
        function [P, Yh] = SampleMeasurementLikelihoodFcn(X, y)
            Q = [   0.3, -0.01
                    -0.01, 0.1 ] * 1e0;
            Yh = ParticleFilter.SampleMeasurementFcn(X);
            P = mvnpdf(y', Yh, Q);
        end
        
        function Xk = ImageUpdateEq(X, dt)
            % X: [x, y, xd, yd]
            % random velocity
            R = diag([10, 10].^2);
            W = mvnrnd([0, 0], R, size(X, 1));

%             dt = 1 / 60;
            Xk = X(:,1:2) + W*dt;
        end
        
        function P = ImageMeasurementLikelihoodFcn(X, y, cameraParams, ...
                                          rotationMatrix, translationVector)
            % y: image
            % X: [x, y, xd, yd] in mm
            P = zeros(size(X, 1), 1);
            for k = 1 : size(X, 1)
                footprint = [X(k,1:2), 0];
                h = 6.5;
                w = 1;  % width/2
                b = 1;
                
%                 boxNot = ([w+b, 0, 0
%                         w+b, 0, h+b
%                         -w-b, 0, h+b
%                         -w-b, 0, 0
%                         -w-b, -b, 0
%                         w+b, -b, 0
%                         w+b, 0, 0] + footprint) * 304.8;
%                 iPts = worldToImage(cameraParams,rotationMatrix,translationVector, boxNot);
%                 BWnot = roipoly(y, iPts(:,1), iPts(:,2)) & y;

                box = ([w, 0, 0
                       w, 0, h
                       -w, 0, h
                       -w, 0, 0
                       w, 0, 0 ] + footprint) * 304.8;
                iPts = worldToImage(cameraParams,rotationMatrix,translationVector, box);
                BW = roipoly(y, iPts(:,1), iPts(:,2)) & y;

                P(k) = sum(sum(y & BW, 1), 2) / sum(BW(:));
                if isnan(P(k))
                    P(k) = 0;
                end
                
                figure, imshow(y), hold on
                fill(iPts(:,1), iPts(:,2), [1, 0, 0], 'linewidth', 1, ...
                    'FaceAlpha', 0.3, 'EdgeColor', [1, 0, 0]);
            end
        end
    end
end

