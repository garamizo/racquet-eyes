%% Try particle filter
Q = [   1, -0.1
        -0.1, 1 ];  % measurement noise
R = 0.001;  % process noise
x = [1; 1];  % initial state
xh = [0, 0];

pf = ParticleFilter(xh + randn(10000, 2));

N = 50;
X = zeros(N, 2);
Y = zeros(N, 2);
U = zeros(N, 1);
Ynoise = zeros(N, 2);
Unoise = zeros(N, 1);
Xh = zeros(N, 2);
Yh = zeros(N, 2);
Px = zeros(2, 2, N);

for k = 1 : N
    % feedback controller
    u = 0.05 * (10*sin(k*0.05) - x(1)) - 0.1 * x(2);
    
    % true state and measurement
    y = ParticleFilter.SampleMeasurementEq(x', [0; 0]);
    X(k,:) = x';
    Y(k,:) = y;
    U(k,:) = u;
    x = ParticleFilter.SampleUpdateEq(x', u, 0)';
    
    % simulate noise
    unoise = u + mvnrnd(0, R)';
    ynoise = y' + mvnrnd([0, 0], Q)';
    Ynoise(k,:) = ynoise';
    Unoise(k,:) = unoise;
    
    % filter (Particle Filter)
    Px(:,:,k) = cov(pf.X);
    [xh, yh] = pf.step(unoise, ynoise);
    Xh(k,:) = xh';
    Yh(k,:) = yh';
    
    % filter KF
    
end

Ybounds = [min(Y); max(Y)];
mean(abs(Y - Yh))
mean(abs(Y - Ynoise))
    
figure, plot(X, ':')
hold on, set(gca, 'ColorOrderIndex', 1), plot(Xh)

figure, 
subplot(121), plot(Ynoise)
hold on, set(gca, 'ColorOrderIndex', 1), plot(Y)
subplot(122), plot(Yh)
hold on, set(gca, 'ColorOrderIndex', 1), plot(Y)

figure,
subplot(221), plot(Ynoise(:,1), Y(:,1), '.', Ybounds(:,1), Ybounds(:,1), 'k'), ylabel('Unfiltered'), axis equal
subplot(222), plot(Ynoise(:,2), Y(:,2), '.', Ybounds(:,2), Ybounds(:,2), 'k'), axis equal
subplot(223), plot(Yh(:,1), Y(:,1), '.', Ybounds(:,1), Ybounds(:,1), 'k'), ylabel('Filtered'), axis equal
subplot(224), plot(Yh(:,2), Y(:,2), '.', Ybounds(:,2), Ybounds(:,2), 'k'), axis equal

%% Test multi-variable normal random
mu = [0, 0];
sigma = [2, -0.5; -0.5, 1];

x = mvnrnd(mu, sigma, 100000);
cov(x);

sigmah = [2, 0.6; 0.6, 1];
yh = mvnpdf(x, mu, sigmah);
y = mvnpdf(x, mu, sigma);

mean(y), mean(yh)

% plot to see ellipse
figure, plot(x(:,1), x(:,2), '.')
axis square

