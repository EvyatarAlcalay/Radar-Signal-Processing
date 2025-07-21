% main.m – Radar Signal Processing (Matched Filter, Delay Estimation, and Target Localization)
% Author: Evyatar Alcalay
% Description:
% Implements matched filtering for radar signal processing,
% including MAP delay estimation, thresholding, and 2D target localization.

%% Question 1 – Basic Delay Estimation using Matched Filter

load('sigvec.mat');
load('delayedvecs.mat');

% Create matched filter coefficients (time-reversed signal)
matchedfiltcoeffs = sigvec(end:-1:1);

% Filter the received signals
filtout_r1 = filter(matchedfiltcoeffs, 1, r1vec);
filtout_r2 = filter(matchedfiltcoeffs, 1, r2vec);

% Plot signals and filter outputs
figure;
subplot(3,1,1); plot(sigvec); title('sigvec');
subplot(3,1,2); plot(r1vec); title('r1vec');
subplot(3,1,3); plot(filtout_r1); title('Matched Filter Output (r1)');

figure;
subplot(3,1,1); plot(sigvec); title('sigvec');
subplot(3,1,2); plot(r2vec); title('r2vec');
subplot(3,1,3); plot(filtout_r2); title('Matched Filter Output (r2)');

% MAP estimators (max of matched filter output)
[~, delay_r1vec] = max(filtout_r1);
[~, delay_r2vec] = max(filtout_r2);

% Adjust for MATLAB filter delay
delay_r1vec = delay_r1vec - length(sigvec);
delay_r2vec = delay_r2vec - length(sigvec);

fprintf('Q1 - MAP delay of r1vec: %d\n', delay_r1vec);
fprintf('Q1 - MAP delay of r2vec: %d\n', delay_r2vec);


%% Question 2 – MAP Estimation with Thresholding (δ < 1)

load('delayedvecsQ2.mat');

% Plot input vectors
figure;
subplot(3,1,1); plot(r1); title('r1'); xlim([0,2050]);
subplot(3,1,2); plot(r2); title('r2'); xlim([0,2050]);
subplot(3,1,3); plot(r3); title('r3'); xlim([0,2050]);

M = length(r1) - length(sigvec); % Number of possible delays
matchedfiltcoeffs = sigvec(end:-1:1);

% Filter
filtout_r1 = filter(matchedfiltcoeffs, 1, r1);
filtout_r2 = filter(matchedfiltcoeffs, 1, r2);
filtout_r3 = filter(matchedfiltcoeffs, 1, r3);

% Calculate MAP thresholds
threshold_r1 = calculate_threshold(sigvec, sigma2_1, delta1, M);
threshold_r2 = calculate_threshold(sigvec, sigma2_2, delta2, M);
threshold_r3 = calculate_threshold(sigvec, sigma2_3, delta3, M);

fprintf('Q2 - threshold_r1: %.2f\n', threshold_r1);
fprintf('Q2 - threshold_r2: %.2f\n', threshold_r2);
fprintf('Q2 - threshold_r3: %.2f\n', threshold_r3);

% Plot filter outputs
figure;
subplot(3,1,1); plot(filtout_r1); title('Matched Filter Output r1'); xlim([0,2050]);
subplot(3,1,2); plot(filtout_r2); title('Matched Filter Output r2'); xlim([0,2050]);
subplot(3,1,3); plot(filtout_r3); title('Matched Filter Output r3'); xlim([0,2050]);

% MAP estimation with thresholding
delay_r1 = delay_estimation(filtout_r1, M, threshold_r1) - length(sigvec);
delay_r2 = delay_estimation(filtout_r2, M, threshold_r2) - length(sigvec);
delay_r3 = delay_estimation(filtout_r3, M, threshold_r3); % r3 might be M (no signal)

fprintf('Q2 - MAP delay of r1: %d\n', delay_r1);
fprintf('Q2 - MAP delay of r2: %d\n', delay_r2);
fprintf('Q2 - MAP delay of r3: %d\n', delay_r3);


%% Question 3 – Radar Target Localization in 2D

load('radarreception.mat');

% Run the radar detection algorithm
[d1vec, d2vec, xvec, yvec] = radardetect(r1vec, r2vec, sigvec);

% Plot the estimated airplane trajectory
figure;
plot(xvec, yvec, 'LineWidth', 2);
title('Plane Location in X-Y coordinates');
xlabel('X [m]');
ylabel('Y [m]');
legend('Plane trajectory');
grid on;

% Calculate velocity vectors
vx = gradient(xvec);
vy = gradient(yvec);

% Plot velocity vectors over path
figure;
quiver(xvec, yvec, vx, vy, 'LineWidth', 1, 'MaxHeadSize', 0.1);
title('Plane Location and Velocity');
xlabel('X [m]');
ylabel('Y [m]');
legend('Velocity vectors');
grid on;


%% --- Supporting Functions ---

function [W_MAP_idx] = delay_estimation(Y_n, M, threshold)
    % Returns MAP estimate of delay given threshold
    [W_MAP_val, W_MAP_idx] = max(Y_n);
    if W_MAP_val < threshold
        W_MAP_idx = M; % signal did not return
    end
end

function threshold = calculate_threshold(sigvec, sigma2, delta, M)
    % Calculates MAP threshold for given signal, noise and return prob
    threshold = (norm(sigvec)^2)/2 + sigma2 * log(((1 - delta)/delta) * (M - 1));
end

function [d1vec, d2vec, xvec, yvec] = radardetect(r1vec, r2vec, sigvec)
    % Estimates target position using two-antenna radar setup

    % Constants
    FS = 5e6;       % samples/sec
    T = 4.096e-4;   % seconds between pulses
    delta = 3840;   % antenna spacing [m]
    c = 3e8;        % speed of light [m/s]

    N = length(r2vec) / (T * FS); % number of pulses

    % Reshape into pulses
    r1vec_resh = reshape(r1vec, [], N);
    r2vec_resh = reshape(r2vec, [], N);

    % Output vectors
    d1vec = zeros(N,1);
    d2vec = zeros(N,1);
    xvec = zeros(N,1);
    yvec = zeros(N,1);

    matchedfiltcoeffs = sigvec(end:-1:1);

    for i = 1:N
        r1_temp = r1vec_resh(:,i);
        r2_temp = r2vec_resh(:,i);

        % Matched filtering
        filtout_r1 = filter(matchedfiltcoeffs, 1, r1_temp);
        filtout_r2 = filter(matchedfiltcoeffs, 1, r2_temp);

        % Delay estimation
        [~, W_1] = max(filtout_r1);
        [~, W_2] = max(filtout_r2);
        W_1 = W_1 - length(sigvec);
        W_2 = W_2 - length(sigvec);
        d1vec(i) = W_1;
        d2vec(i) = W_2;

        % Convert to distance
        d1 = (W_1 * c) / FS;
        d2 = (W_2 * c) / FS;

        % Localization equations
        A = 0.25 * d1^2;
        B = (d2 - 0.5 * d1)^2;
        xvec(i) = (A - B)/(2 * delta) + delta / 2;
        yvec(i) = sqrt(max(0, A - xvec(i)^2));  % avoid complex result
    end
end
