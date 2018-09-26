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
    end
end

