% ===============================================================
%  FILE NAME:      music_multipath_exp1.m
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

close all;
clc;
clear;

% Parameters
M = 16;             % Number of array elements
d = 0.5;            % Spacing between elements (in lambda)
lambda = 1;         % Wavelength
% important role of Snapshots in the MUSIC regarding coherence i.e.
% uncorrelated signal from the diff sources
N = 1000;           % Number of snapshots % coherence will play the role
SNR = 20;           % Signal-to-noise ratio

% Case 1: Two independent targets at 30° and 40°
angles_independent = [30, 40];   % Degrees
theta_independent = angles_independent * pi / 180;   % Convert to radians

steering_matrix_independent = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta_independent));

% Generate independent signals
signal_independent = (randn(2, N) + 1j * randn(2, N)) / sqrt(2);
X_independent = steering_matrix_independent * signal_independent;  % adding random intial phase for the signal from the diff sources

% Case 2: One target at 30°, multipath reflected from 40°
angles_multipath = [30, 40];   % Degrees
theta_multipath = angles_multipath * pi / 180;   % Convert to radians
steering_matrix_multipath = exp(-1j * 2 * pi * d * (0:M-1).' * sin(theta_multipath));

% Generate multipath signals (coherent sources)
signal_multipath = (randn(1, N) + 1j * randn(1, N)) / sqrt(2);
X_multipath = steering_matrix_multipath * [signal_multipath; signal_multipath];  % Same signal

% Add noise
X_independent = awgn(X_independent, SNR, 'measured');
X_multipath = awgn(X_multipath, SNR, 'measured');

% Covariance matrices
R_independent = (X_independent * X_independent') / N;
R_multipath = (X_multipath * X_multipath') / N;

% MUSIC for independent targets
[~, P_music_independent] = music_spectrum(R_independent, M, d, lambda, 2);

% MUSIC for multipath case (without smoothing)
[~, P_music_multipath] = music_spectrum(R_multipath, M, d, lambda, 2);

% Analyze multipath case without smoothing : wrong , not working
peakThr = -10; 
% [isMultipath_multipath, numDetectedSources_multipath] = isMultipathMusic(R_multipath, M, d, lambda, 1, peakThr);


% Spatial smoothing
M_smooth = 8;
R_smoothed = spatial_smoothing(X_multipath, M, M_smooth);  % 8 subarrays

% MUSIC for multipath case (with spatial smoothing)
[~, P_music_multipath_smoothed] = music_spectrum(R_smoothed, M_smooth, d, lambda, 2);

% Plot the results
figure;
subplot(3,1,1);
plot(-90:0.1:90, P_music_independent, 'LineWidth', 2);
title('MUSIC Spectrum for Two Independent Targets');
xlabel('Angle (degrees)');
ylabel('Spatial Spectrum (dB)');
grid on;

subplot(3,1,2);
plot(-90:0.1:90, P_music_multipath, 'LineWidth', 2);
title('MUSIC Spectrum for Multipath Case (Coherent Signals)');
xlabel('Angle (degrees)');
ylabel('Spatial Spectrum (dB)');
grid on;

subplot(3,1,3);
plot(-90:0.1:90, P_music_multipath_smoothed, 'LineWidth', 2);
title('MUSIC Spectrum for Multipath Case (With Spatial Smoothing)');
xlabel('Angle (degrees)');
ylabel('Spatial Spectrum (dB)');
grid on;

