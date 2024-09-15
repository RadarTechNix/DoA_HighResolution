% ===============================================================
%  FILE NAME:      music_exp1.m
%  AUTHOR:         Nikhil Navghade 
%  DATE CREATED:   15-Sep-2024
%  LAST MODIFIED:  15-Sep-2024
%  ORGANIZATION:   Independent Developer
%  VERSION:        1.0
%
% LICENSE: MIT License
% Permission is granted, free of charge, to use, copy, modify, and distribute
% this software for any purpose, with or without attribution. The software is
% provided "as-is" without warranty of any kind, express or implied.
%
%  ===============================================================

% MATLAB Code for DOA Estimation using MUSIC Algorithm

close all;
clc;
clear;

% Parameters
M = 16;          
% Number of array elements (antennas)
d = 0.5;            % Spacing between elements in multiples of wavelength (0.5 lambda)
lambda = 1;         % Wavelength (normalized to 1)
N = 1%1000;           % Number of snapshots
K = 2;              % Number of sources (assume 2 sources for example)

% Generate synthetic data (for testing)
% Assuming two signals coming from angles -20 and 40 degrees
angles = [-20, 40];
signal_power = [1, 0.5];
noise_power = 0.01;

% Generate array response matrix
theta = angles(:) * pi/180; % Convert to radians
steering_matrix = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta)');

% Generate received signal
signals = (sqrt(signal_power(:)) .* (randn(K, N) + 1j*randn(K, N))) / sqrt(2);
X_nl = steering_matrix * signals; % Noiseless received signal

% Add noise
X = X_nl + sqrt(noise_power/2) * (randn(M, N) + 1j*randn(M, N));

% Compute covariance matrix
R = (X * X') / N;

%%
%{
 [V,D] = eig(A) produces a diagonal matrix D of eigenvalues and 
    a full matrix V whose columns are the corresponding eigenvectors  
    so that A*V = V*D. 
%}
% Eigenvalue decomposition
[E, D] = eig(R);

% Sort eigenvalues and corresponding eigenvectors
[eigenvalues, idx] = sort(diag(D), 'descend');
E = E(:, idx);

% Signal subspace: first K eigenvectors
En = E(:, K+1:end);  % Noise subspace (remaining M-K eigenvectors)

% MUSIC Spectrum Calculation
theta_scan = -90:0.1:90;  % Scan angles from -90 to 90 degrees
P_music = zeros(size(theta_scan));

for i = 1:length(theta_scan)
    a_theta = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta_scan(i) * pi/180));
    P_music(i) = 1 / (a_theta' * En * En' * a_theta);
end

% Convert to dB scale
P_music = 10*log10(abs(P_music));

% Plot MUSIC Spectrum
figure;
plot(theta_scan, P_music, 'LineWidth', 2);
xlabel('Angle (degrees)');
ylabel('Spatial Spectrum (dB)');
title('MUSIC Spectrum');
grid on;

nope;
