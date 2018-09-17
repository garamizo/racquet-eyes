classdef ParticleFilter < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    %   Xkk = A*Xk + B*uk + vk
    %   Y = C*Xk + wk
    
    properties
        X  % state particles
        measurementEq
        updateEq
    end
    
    methods
        function obj = ParticleFilter(X0)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.X = X0;
            obj.measurementEq = @ParticleFilter.SampleMeasurementEq;
            obj.updateEq = @ParticleFilter.SampleUpdateEq;
        end
        
        function [xh, yh] = step(obj, u, y)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % calculate posterior probability
            [Yh, P] = obj.measurementEq(obj.X, y);
            
            % select best estimates
            [~, midx] = max(P);
            xh = obj.X(midx,:)';
            yh = Yh(midx,:)';
            
            % resample states
            Ps = cumsum(P);
            nselect = rand(1, length(Ps)) * Ps(end);
            [~, idx] = min(abs(Ps - nselect));
            obj.X = obj.X(idx,:);
            
            % update state
            obj.X = obj.updateEq(obj.X, u);
            
%             mean(P(idx))
%             mean(P)
        end
    end
    
    methods (Static)
        function Xk = SampleUpdateEq(X, u, V)
            % p(Xkk | uk, Xk)
            if nargin == 2
                R = 0.001;
                V = mvnrnd(0, R, size(X, 1));
            end
           
            A = [   1, 1
                    0, 1 ];
            B = [   0
                    1 ];
            
            Xk = X * A' + (u' + V) * B';
        end
        
        function [Yh, P] = SampleMeasurementEq(X, y)
            % P(z | X)
            C = [   1, 0
                    0, 1 ];
            Q = [   1, -0.1
                    -0.1, 1 ];
                
            Yh = X * C';
            P = mvnpdf(y', Yh, Q);
        end
    end
end

